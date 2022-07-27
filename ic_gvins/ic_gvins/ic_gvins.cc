/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "ic_gvins.h"
#include "misc.h"

#include "common/angle.h"
#include "common/earth.h"
#include "common/gpstime.h"
#include "common/logging.h"

#include "factors/gnss_factor.h"
#include "factors/marginalization_factor.h"
#include "factors/marginalization_info.h"
#include "factors/pose_parameterization.h"
#include "factors/reprojection_factor.h"
#include "factors/residual_block_info.h"
#include "preintegration/imu_error_factor.h"
#include "preintegration/preintegration.h"
#include "preintegration/preintegration_factor.h"

#include <ceres/ceres.h>
#include <yaml-cpp/yaml.h>

GVINS::GVINS(const string &configfile, const string &outputpath, Drawer::Ptr drawer) {
    gvinsstate_ = GVINS_ERROR;

    // 加载配置
    // Load configuration
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }

    // 文件IO
    // Output files
    navfilesaver_    = FileSaver::create(outputpath + "/gvins.nav", 11);
    ptsfilesaver_    = FileSaver::create(outputpath + "/mappoint.txt", 3);
    statfilesaver_   = FileSaver::create(outputpath + "/statistics.txt", 3);
    extfilesaver_    = FileSaver::create(outputpath + "/extrinsic.txt", 3);
    imuerrfilesaver_ = FileSaver::create(outputpath + "/IMU_ERR.bin", 7, FileSaver::BINARY);
    trajfilesaver_   = FileSaver::create(outputpath + "/trajectory.csv", 8);

    if (!navfilesaver_->isOpen() || !ptsfilesaver_->isOpen() || !statfilesaver_->isOpen() || !extfilesaver_->isOpen()) {
        LOGE << "Failed to open data file";
        return;
    }

    // Make a copy of configuration file to the output directory
    std::ofstream ofconfig(outputpath + "/gvins.yaml");
    ofconfig << YAML::Dump(config);
    ofconfig.close();

    initlength_       = config["initlength"].as<int>();
    imudatarate_      = config["imudatarate"].as<double>();
    imudatadt_        = 1.0 / imudatarate_;
    reserved_ins_num_ = 2;

    // 安装参数
    // Installation parameters
    vecdata    = config["antlever"].as<std::vector<double>>();
    antlever_  = Vector3d(vecdata.data());

    // IMU噪声参数
    // IMU parameters
    integration_parameters_               = std::make_shared<IntegrationParameters>();
    integration_parameters_->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
    integration_parameters_->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
    integration_parameters_->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
    integration_parameters_->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
    integration_parameters_->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;
    integration_parameters_->gravity      = NORMAL_GRAVITY;

    integration_config_.iswithearth = config["iswithearth"].as<bool>();
    integration_config_.isuseodo    = false;
    integration_config_.iswithscale = false;
    integration_config_.gravity     = {0, 0, NORMAL_GRAVITY};

    // 初始值, 后续根据GNSS定位实时更新
    // GNSS variables intializaiton
    integration_config_.origin.setZero();
    last_gnss_.blh.setZero();
    gnss_.blh.setZero();

    preintegration_options_ = Preintegration::getOptions(integration_config_);

    // 相机参数
    // Camera parameters
    vector<double> intrinsic  = config["cam0"]["intrinsic"].as<std::vector<double>>();
    vector<double> distortion = config["cam0"]["distortion"].as<std::vector<double>>();
    vector<int> resolution    = config["cam0"]["resolution"].as<std::vector<int>>();

    camera_ = Camera::createCamera(intrinsic, distortion, resolution);

    // IMU和Camera外参
    // Extrinsic parameters
    vecdata           = config["cam0"]["q_b_c"].as<std::vector<double>>();
    Quaterniond q_b_c = Eigen::Quaterniond(vecdata.data());
    vecdata           = config["cam0"]["t_b_c"].as<std::vector<double>>();
    Vector3d t_b_c    = Eigen::Vector3d(vecdata.data());
    td_b_c_           = config["cam0"]["td_b_c"].as<double>();

    pose_b_c_.R = q_b_c.toRotationMatrix();
    pose_b_c_.t = t_b_c;

    // 优化参数
    // Optimization parameters
    reprojection_error_std_      = config["reprojection_error_std"].as<double>();
    optimize_estimate_extrinsic_ = config["optimize_estimate_extrinsic"].as<bool>();
    optimize_estimate_td_        = config["optimize_estimate_td"].as<bool>();
    optimize_num_iterations_     = config["optimize_num_iterations"].as<int>();
    optimize_windows_size_       = config["optimize_windows_size"].as<size_t>();

    // 归一化相机坐标系下
    // Reprojection std
    optimize_reprojection_error_std_ = reprojection_error_std_ / camera_->focalLength();

    // 可视化
    is_use_visualization_ = config["is_use_visualization"].as<bool>();

    // Initialize the containers
    preintegrationlist_.clear();
    statedatalist_.clear();
    gnsslist_.clear();
    timelist_.clear();

    // GVINS fusion objects
    map_    = std::make_shared<Map>(optimize_windows_size_);
    drawer_ = std::move(drawer);
    drawer_->setMap(map_);
    if (is_use_visualization_) {
        drawer_thread_ = std::thread(&Drawer::run, drawer_);
    }
    tracking_ = std::make_shared<Tracking>(camera_, map_, drawer_, configfile, outputpath);

    // Process threads
    fusion_thread_       = std::thread(&GVINS::runFusion, this);
    tracking_thread_     = std::thread(&GVINS::runTracking, this);
    optimization_thread_ = std::thread(&GVINS::runOptimization, this);

    gvinsstate_ = GVINS_INITIALIZING;
}

bool GVINS::addNewImu(const IMU &imu) {
    if (imu_buffer_mutex_.try_lock()) {
        if (imu.dt > (imudatadt_ * 1.5)) {
            LOGE << absl::StrFormat("Lost IMU data with at %0.3lf dt %0.3lf", imu.time, imu.dt);

            long cnts = lround(imu.dt / imudatadt_) - 1;

            IMU imudata  = imu;
            imudata.time = imu.time - imu.dt;
            while (cnts--) {
                imudata.time += imudatadt_;
                imudata.dt = imudatadt_;
                imu_buffer_.push(imudata);
                LOGE << "Append extra IMU data at " << Logging::doubleData(imudata.time);
            }
        } else {
            imu_buffer_.push(imu);
        }

        // 释放信号量
        // Release fusion semaphore
        fusion_sem_.notify_one();

        imu_buffer_mutex_.unlock();
        return true;
    }

    return false;
}

bool GVINS::addNewGnss(const GNSS &gnss) {
    // 低频观测, 无需加锁

    // 根据GNSS定位更新重力常量
    // Update the gravity from GNSS
    if (integration_config_.origin.isZero()) {
        // 站心原点
        // The origin of the world frame
        integration_config_.origin       = gnss.blh;
        integration_parameters_->gravity = Earth::gravity(gnss.blh);
        LOGI << "Local gravity is initialized as " << Logging::doubleData(integration_parameters_->gravity);
    } else {
        last_last_gnss_ = last_gnss_;
        last_gnss_      = gnss_;
    }

    gnss_        = gnss;
    gnss_.blh    = Earth::global2local(integration_config_.origin, gnss_.blh);
    isgnssready_ = true;

    return true;
}

bool GVINS::addNewFrame(const Frame::Ptr &frame) {
    if (gvinsstate_ > GVINS_INITIALIZING_INS) {
        if (frame_buffer_mutex_.try_lock()) {
            frame_buffer_.push(frame);

            tracking_sem_.notify_one();

            frame_buffer_mutex_.unlock();
            return true;
        }
        return false;
    }
    return true;
}

void GVINS::runFusion() {
    IMU imu_pre, imu_cur;
    IntegrationState state;
    Frame::Ptr frame;

    LOGI << "Fusion thread is started";
    while (!isfinished_) { // While
        Lock lock(fusion_mutex_);
        fusion_sem_.wait(lock);

        // 获取所有有效数据
        // Process all IMU data
        while (!imu_buffer_.empty()) { // IMU BUFFER
            // 读取IMU缓存
            // Load an IMU sample
            {
                Lock lock2(imu_buffer_mutex_);
                imu_pre = imu_cur;
                imu_cur = imu_buffer_.front();
                imu_buffer_.pop();
            }

            // INS机械编排及INS处理
            // INS mechanization
            { // INS
                Lock lock3(ins_mutex_);
                if (!ins_window_.empty()) {
                    // 上一时刻的状态
                    // The INS state in last time for mechanization
                    state = ins_window_.back().second;
                }
                ins_window_.emplace_back(imu_cur, IntegrationState());

                // 初始化完成后开始积分输出
                if (gvinsstate_ > GVINS_INITIALIZING) {
                    if (isoptimized_ && state_mutex_.try_lock()) {
                        // 优化求解结束, 需要更新IMU误差重新积分
                        // When the optimization is finished
                        isoptimized_ = false;

                        state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
                        MISC::redoInsMechanization(integration_config_, state, reserved_ins_num_, ins_window_);

                        state_mutex_.unlock();
                    } else {
                        // 单次机械编排
                        // Do a single INS mechanization
                        MISC::insMechanization(integration_config_, imu_pre, imu_cur, state);

                        ins_window_.back().second = state;
                    }
                } else {
                    // Only reserve certain INS in the window during initialization
                    if (ins_window_.size() > MAXIMUM_INS_NUMBER) {
                        ins_window_.pop_front();
                    }
                }

                // 融合状态
                // Fusion process
                if (gvinsstate_ == GVINS_INITIALIZING) {
                    if (isgnssready_ && state_mutex_.try_lock()) {
                        // 初始化参数
                        // GVINS initialization using GNSS/INS initialization
                        if (gvinsInitialization()) {
                            gvinsstate_ = GVINS_INITIALIZING_INS;

                            // 初始化时需要重新积分
                            // Redo INS mechanization
                            isoptimized_ = true;
                        }
                        isgnssready_ = false;

                        state_mutex_.unlock();
                        continue;
                    }
                } else if (gvinsstate_ == GVINS_INITIALIZING_INS) {
                    // 新的GNSS观测到来, 进行优化
                    // New GNSS, do GNSS/INS integration
                    if (isgnssready_ && state_mutex_.try_lock()) {
                        // 需要保证数据对齐, 否则等待
                        // For data align
                        if (gnss_.time < ins_window_.back().first.time) {

                            // 加入新的GNSS节点
                            // Add a new GNSS time node
                            addNewGnssTimeNode();

                            isgnssready_ = false;
                            isgnssobs_   = true;
                            optimization_sem_.notify_one();
                        }
                        state_mutex_.unlock();
                    }
                } else if (gvinsstate_ == GVINS_INITIALIZING_VIO) {
                    // 仅加入关键帧节点, 而不进行优化
                    // Add new time node during the initialization of the visual system
                    if ((isframeready_ || isgnssready_) && state_mutex_.try_lock()) {
                        if (isframeready_ && (keyframes_.front()->stamp() < ins_window_.back().first.time)) {
                            addNewKeyFrameTimeNode();

                            isframeready_ = false;

                            gvinsstate_ = GVINS_TRACKING_INITIALIZING;
                        }

                        // 如果有GNSS观测, 也要添加节点
                        // Add GNSS if available
                        if (isgnssready_) {
                            if (insertNewGnssTimeNode()) {
                                isgnssready_ = false;
                            }
                        }

                        state_mutex_.unlock();
                    }
                } else if (gvinsstate_ >= GVINS_TRACKING_INITIALIZING) {
                    if ((isframeready_ || isgnssready_) && state_mutex_.try_lock()) {
                        if (isframeready_ && (keyframes_.front()->stamp() < ins_window_.back().first.time)) {
                            addNewKeyFrameTimeNode();

                            isframeready_ = false;
                            isvisualobs_  = true;
                        }

                        // 如果有GNSS观测
                        // Add GNSS if available
                        if (isgnssready_) {
                            if (insertNewGnssTimeNode()) {
                                isgnssready_ = false;
                                isgnssobs_   = true;
                            }
                        }

                        state_mutex_.unlock();

                        // Release the optimization semaphore
                        if (isvisualobs_) {
                            optimization_sem_.notify_one();
                        }
                    }
                }

                // 用于输出
                // For output only
                state = ins_window_.back().second;
            } // INS

            // 总是输出最新的INS机械编排结果, 不占用INS锁
            // Always output the INS results
            if (gvinsstate_ > GVINS_INITIALIZING) {
                MISC::writeNavResult(integration_config_, state, navfilesaver_, imuerrfilesaver_, trajfilesaver_);
            }

        } // IMU BUFFER
    }     // While
}

void GVINS::runOptimization() {

    TimeCost timecost, timecost2;

    LOGI << "Optimization thread is started";
    while (!isfinished_) {
        Lock lock(optimization_mutex_);
        optimization_sem_.wait(lock);

        if (isgnssobs_ || isvisualobs_) {
            timecost.restart();

            // 加锁, 保护状态量
            // Lock the state
            state_mutex_.lock();

            if (gvinsstate_ == GVINS_INITIALIZING_INS) {
                // GINS优化
                // GNSS/INS optimization
                bool isinitialized = gvinsInitializationOptimization();

                if (preintegrationlist_.size() >= static_cast<size_t>(initlength_)) {
                    // 完成GINS初始化, 进入视觉初始化阶段
                    // Enter the initialization of the visual system
                    gvinsstate_ = GVINS_INITIALIZING_VIO;
                    if (isinitialized) {
                        LOGI << "GINS initialization is finished";
                    } else {
                        LOGW << "GINS initialization is not convergence";
                    }
                }
            } else if (gvinsstate_ >= GVINS_TRACKING_INITIALIZING) {

                if (map_->isMaximumKeframes()) {
                    gvinsstate_ = GVINS_TRACKING_NORMAL;
                }

                // 两次非线性优化并进行粗差剔除
                // Two-steps optimization with outlier culling
                gvinsOptimization();

                timecost2.restart();

                // 移除所有窗口中间插入的非关键帧
                // Remove all non-keyframes time nodes
                gvinsRemoveAllSecondNewFrame();

                // 关键帧数量达到窗口大小, 需要边缘化操作, 并移除最老的关键帧及相关的GNSS和预积分观测, 由于计算力的问题,
                // 可能导致多个关键帧同时加入优化, 需要进行多次边缘化操作
                // Do marginalization
                while (map_->isMaximumKeframes()) {
                    // 边缘化, 移除旧的观测, 按时间对齐到保留的最后一个关键帧
                    gvinsMarginalization();
                }

                timecosts_[2] = timecost2.costInMillisecond();

                // 统计并输出视觉相关的参数
                // Log the statistic parameters
                parametersStatistic();
            }

            // 可视化
            // For visualization
            if (is_use_visualization_) {
                auto state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
                drawer_->updateMap(MISC::pose2Twc(MISC::stateToCameraPose(state, pose_b_c_)));
            }

            if (isgnssobs_)
                isgnssobs_ = false;
            if (isvisualobs_)
                isvisualobs_ = false;

            // Release the state lock
            state_mutex_.unlock();
            isoptimized_ = true;

            LOGI << "Optimization costs " << timecost.costInMillisecond() << " ms with " << timecosts_[0] << " and "
                 << timecosts_[1] << " with marginalization costs " << timecosts_[2];
        }
    }
}

void GVINS::runTracking() {
    Frame::Ptr frame;
    Pose pose;
    IntegrationState state, state0, state1;

    std::deque<std::pair<IMU, IntegrationState>> ins_windows;

    LOGI << "Tracking thread is started";
    while (!isfinished_) {
        Lock lock(tracking_mutex_);
        tracking_sem_.wait(lock);

        // 处理所有缓存
        // Process all the frames
        while (!frame_buffer_.empty()) {
            TimeCost timecost;

            Pose pose_b_c;
            double td;
            {
                Lock lock3(extrinsic_mutex_);
                pose_b_c = pose_b_c_;
                td       = td_b_c_;
            }

            // 读取缓存
            {
                frame_buffer_mutex_.lock();
                frame = frame_buffer_.front();

                // 保证每个图像都有先验的惯导位姿
                // Wait until the INS is available
                ins_mutex_.lock();
                if (ins_window_.empty() || (ins_window_.back().second.time <= (frame->stamp() + td))) {
                    ins_mutex_.unlock();
                    frame_buffer_mutex_.unlock();

                    usleep(1000);
                    continue;
                }
                ins_mutex_.unlock();

                frame_buffer_.pop();
                frame_buffer_mutex_.unlock();
            }

            // 获取初始位姿
            // The prior pose from INS
            {
                Lock lock2(ins_mutex_);
                frame->setStamp(frame->stamp() + td);
                frame->setTimeDelay(td);
                MISC::getCameraPoseFromInsWindow(ins_window_, pose_b_c, frame->stamp(), pose);
                frame->setPose(pose);
            }

            TrackState trackstate = tracking_->track(frame);
            if (trackstate == TRACK_LOST) {
                LOGE << "Tracking lost at " << Logging::doubleData(frame->stamp());
            }

            // 包括第一帧在内的所有关键帧, 跟踪失败时的当前帧也会成为新的关键帧
            // All possible keyframes
            if (tracking_->isNewKeyFrame() || (trackstate == TRACK_FIRST_FRAME) || trackstate == TRACK_LOST) {
                Lock lock3(keyframes_mutex_);
                keyframes_.push(frame);

                isframeready_ = true;

                LOGI << "Tracking cost " << timecost.costInMillisecond() << " ms";
            }
        }
    }
}

void GVINS::setFinished() {
    isfinished_ = true;

    // 释放信号量, 退出所有线程
    // Release all semaphores
    fusion_sem_.notify_all();
    tracking_sem_.notify_all();
    optimization_sem_.notify_all();

    tracking_thread_.join();
    optimization_thread_.join();
    fusion_thread_.join();

    if (is_use_visualization_) {
        drawer_->setFinished();
        drawer_thread_.join();
    }

    Quaterniond q_b_c = Rotation::matrix2quaternion(pose_b_c_.R);
    Vector3d t_b_c    = pose_b_c_.t;

    LOGW << "GVINS has finished processing";
    LOGW << "Estimated extrinsics: "
         << absl::StrFormat("(%0.6lf, %0.6lf, %0.6lf, %0.6lf), (%0.3lf, %0.3lf, "
                            "%0.3lf), %0.4lf",
                            q_b_c.x(), q_b_c.y(), q_b_c.z(), q_b_c.w(), t_b_c.x(), t_b_c.y(), t_b_c.z(), td_b_c_);

    Logging::shutdownLogging();
}

bool GVINS::gvinsInitialization() {

    if ((gnss_.time == 0) || (last_gnss_.time == 0)) {
        return false;
    }

    // 缓存数据用于零速检测
    // Buffer for zero-velocity detection
    vector<IMU> imu_buff;
    for (const auto &ins : ins_window_) {
        auto &imu = ins.first;
        if ((imu.time > last_gnss_.time) && (imu.time < gnss_.time)) {
            imu_buff.push_back(imu);
        }
    }
    if (imu_buff.size() < 20) {
        return false;
    }

    // 零速检测估计陀螺零偏和横滚俯仰角
    // Obtain the gyroscope biases and roll and pitch angles
    vector<double> average;
    static Vector3d bg{0, 0, 0};
    static Vector3d initatt{0, 0, 0};
    bool is_zero_velocity = MISC::detectZeroVelocity(imu_buff, imudatarate_, average);
    if (is_zero_velocity) {
        // 陀螺零偏
        bg = Vector3d(average[0], average[1], average[2]);
        bg *= imudatarate_;

        // 重力调平获取横滚俯仰角
        Vector3d fb(average[3], average[4], average[5]);
        fb *= imudatarate_;

        initatt[0] = -asin(fb[1] / integration_parameters_->gravity);
        initatt[1] = asin(fb[0] / integration_parameters_->gravity);

        LOGI << "Zero velocity get gyroscope bias " << bg.transpose() * 3600 * R2D << ", roll " << initatt[0] * R2D
             << ", pitch " << initatt[1] * R2D;
    }

    // 里程计速度大于MINMUM_ALIGN_VELOCITY, 或者非零速状态
    // Initialization conditions
    Vector3d velocity = Vector3d::Zero();
    if ((integration_config_.isuseodo && imu_buff.back().odovel > MINMUM_ALIGN_VELOCITY) || !is_zero_velocity) {
        if (last_gnss_.isyawvalid) {
            initatt[2] = last_gnss_.yaw;
            LOGI << "Initialized heading from dual-antenna GNSS as " << initatt[2] * R2D << " deg";
        } else {
            velocity = gnss_.blh - last_gnss_.blh;
            if (velocity.norm() < MINMUM_ALIGN_VELOCITY) {
                return false;
            }
            initatt[2] = atan2(velocity.y(), velocity.x());
            LOGI << "Initialized heading from GNSS positioning as " << initatt[2] * R2D << " deg";
        }
    } else {
        return false;
    }

    // 初始状态, 从上一秒开始
    // The initialization state
    auto state = IntegrationState{
        .time = last_gnss_.time,
        .p    = last_gnss_.blh - Rotation::euler2quaternion(initatt) * antlever_,
        .q    = Rotation::euler2quaternion(initatt),
        .v    = velocity,
        .bg   = bg,
        .ba   = {0, 0, 0},
        .sodo = 0.0,
        .sg   = {0, 0, 0},
        .sa   = {0, 0, 0},
    };
    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));
    gnsslist_.push_back(last_gnss_);
    timelist_.push_back(last_gnss_.time);

    // 初始化重力和地球自转参数
    // The gravity and the Earth rotation rate
    integration_config_.gravity = Vector3d(0, 0, integration_parameters_->gravity);
    if (integration_config_.iswithearth) {
        integration_config_.iewn = Earth::iewn(integration_config_.origin, state.p);
    }

    // 计算第一秒的INS结果
    // Redo INS mechanization at the first second
    state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);
    MISC::redoInsMechanization(integration_config_, state, reserved_ins_num_, ins_window_);

    LOGI << "Initialization at " << Logging::doubleData(gnss_.time);

    // 加入当前GNSS时间节点
    // Add current GNSS time node
    addNewGnssTimeNode();

    return true;
}

bool GVINS::gvinsInitializationOptimization() {
    // GNSS/INS optimization

    // 构建优化问题
    ceres::Solver solver;
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations         = 50;

    // 参数块
    // Add parameter blocks
    addStateParameters(problem);

    // GNSS残差
    // Add gnss factors
    addGnssFactors(problem, true);

    // 预积分残差
    // Add IMU preintegration factors
    addImuFactors(problem);

    solver.Solve(options, &problem, &summary);
    LOGI << summary.BriefReport();

    return summary.termination_type == ceres::CONVERGENCE;
}

void GVINS::addNewKeyFrameTimeNode() {

    Lock lock(keyframes_mutex_);
    vector<double> extra_gnss_node;
    while (!keyframes_.empty()) {
        // 取出一个关键帧
        // Obtain a new valid keyframe
        auto frame       = keyframes_.front();
        double frametime = frame->stamp();
        if (frametime > ins_window_.back().first.time) {
            break;
        }

        keyframes_.pop();

        // 添加关键帧
        // Add new keyframe time node
        LOGI << "Insert keyframe " << frame->keyFrameId() << " at " << Logging::doubleData(frame->stamp()) << " with "
             << frame->unupdatedMappoints().size() << " new mappoints";
        map_->insertKeyFrame(frame);

        addNewTimeNode(frametime);
        LOGI << "Add new keyframe time node at " << Logging::doubleData(frametime);
    }

    // 移除多余的预积分节点
    // Remove unused time node
    removeUnusedTimeNode();
}

bool GVINS::removeUnusedTimeNode() {
    if (unused_time_nodes_.empty()) {
        return false;
    }

    LOGI << "Remove " << unused_time_nodes_.size() << " unused time node "
         << Logging::doubleData(unused_time_nodes_[0]);

    for (double node : unused_time_nodes_) {
        int index = getStateDataIndex(node);

        // Exception
        if (index < 0) {
            continue;
        }

        auto first_preintegration  = preintegrationlist_[index - 1];
        auto second_preintegration = preintegrationlist_[index];
        auto imu_buffer            = second_preintegration->imuBuffer();

        // 将后一个预积分的IMU数据合并到前一个, 不包括第一个IMU数据
        // Merge the IMU preintegration
        for (size_t k = 1; k < imu_buffer.size(); k++) {
            first_preintegration->addNewImu(imu_buffer[k]);
        }

        // 移除时间节点, 以及后一个预积分
        // Remove the second time node
        preintegrationlist_.erase(preintegrationlist_.begin() + index);
        timelist_.erase(timelist_.begin() + index);
        statedatalist_.erase(statedatalist_.begin() + index);
    }
    unused_time_nodes_.clear();

    return true;
}

bool GVINS::insertNewGnssTimeNode() {
    // Wait new keyframe to determine GNSS time ndoe
    if (gnss_.time > timelist_.back()) {
        return false;
    }

    // Find time interval
    double sta = 0, end = 0;
    size_t index = 0;
    for (size_t k = timelist_.size() - 1; k > 1; k--) {
        if ((gnss_.time <= timelist_[k]) && (gnss_.time > timelist_[k - 1])) {
            sta   = timelist_[k - 1];
            end   = timelist_[k];
            index = k;
        }
    }
    if (sta == 0) {
        return false;
    }

    // Check the keyframe is a normal keyframe
    bool is_need_gnss = false;
    auto keyframeids  = map_->orderedKeyFrames();
    for (int k = keyframeids.size() - 1; k >= 0; k--) {
        auto frame = map_->keyframes().find(keyframeids[k])->second;

        double keyframe_time = frame->stamp();
        if (MISC::isTheSameTimeNode(keyframe_time, end, MISC::MINIMUM_TIME_INTERVAL)) {
            if (frame->keyFrameState() != KEYFRAME_REMOVE_SECOND_NEW) {
                is_need_gnss = true;
            }
        }
    }
    if (!is_need_gnss) {
        LOGI << "Unused GNSS due to non-normal keyframe at " << Logging::doubleData(gnss_.time);
        return true;
    }

    if (gnss_.time - sta < MINMUM_SYNC_INTERVAL) {
        // Align to previous node
        GNSS gnss = gnss_;
        gnss.time = sta;

        // Compensate the time
        double dt = gnss_.time - sta;
        gnss.blh[0] -= statedatalist_[index - 1].mix[0] * dt;
        gnss.blh[1] -= statedatalist_[index - 1].mix[1] * dt;
        gnss.blh[2] -= statedatalist_[index - 1].mix[2] * dt;
        gnss.std *= 1.2;

        gnsslist_.push_back(gnss);
        LOGI << "Add new GNSS " << Logging::doubleData(gnss_.time) << " align to " << Logging::doubleData(sta);
    } else if (end - gnss_.time < MINMUM_SYNC_INTERVAL) {
        // Align to current node
        GNSS gnss = gnss_;
        gnss.time = end;

        // Compensate the time
        double dt = end - gnss_.time;
        gnss.blh[0] += statedatalist_[index].mix[0] * dt;
        gnss.blh[1] += statedatalist_[index].mix[1] * dt;
        gnss.blh[2] += statedatalist_[index].mix[2] * dt;
        gnss.std *= 1.2;

        gnsslist_.push_back(gnss);
        LOGI << "Add new GNSS " << Logging::doubleData(gnss_.time) << " align to " << Logging::doubleData(end);
    } else {
        // Avoid reintegrating the long-time preintegration
        if (preintegrationlist_[index - 1]->deltaTime() > MAXIMUM_PREINTEGRATION_LENGTH) {
            LOGI << "Unused GNSS due to long-time preintegration " << Logging::doubleData(gnss_.time);
            return true;
        }

        // Insert GNSS node to sliding window
        vector<double> timelist;
        for (size_t k = index; k < timelist_.size(); k++) {
            timelist.push_back(timelist_[k]);
        }

        // Remove back time node
        size_t num_remove = timelist_.size() - index;
        for (size_t k = num_remove; k > 0; k--) {
            timelist_.pop_back();
            statedatalist_.pop_back();
            preintegrationlist_.pop_back();
        }

        // Add GNSS time node
        addNewGnssTimeNode();

        // Add back time node
        for (size_t k = 0; k < timelist.size(); k++) {
            addNewTimeNode(timelist[k]);
        }
    }

    return true;
}

void GVINS::addNewGnssTimeNode() {
    LOGI << "Add new GNSS time node " << Logging::doubleData(gnss_.time);

    addNewTimeNode(gnss_.time);
    gnsslist_.push_back(gnss_);
}

void GVINS::addNewTimeNode(double time) {

    vector<IMU> series;
    IntegrationState state;

    // 获取时段内用于预积分的IMU数据
    // Obtain the IMU samples between the two time nodes
    double start = timelist_.back();
    double end   = time;
    MISC::getImuSeriesFromTo(ins_window_, start, end, series);

    state = Preintegration::stateFromData(statedatalist_.back(), preintegration_options_);

    // 新建立新的预积分
    // Build a new IMU preintegration
    preintegrationlist_.emplace_back(
        Preintegration::createPreintegration(integration_parameters_, series[0], state, preintegration_options_));

    // 预积分, 从第二个历元开始
    // Add IMU sample
    for (size_t k = 1; k < series.size(); k++) {
        preintegrationlist_.back()->addNewImu(series[k]);
    }

    // 当前状态加入到滑窗中
    // Add current state and time node to the sliding window
    state      = preintegrationlist_.back()->currentState();
    state.time = time;

    statedatalist_.emplace_back(Preintegration::stateToData(state, preintegration_options_));
    timelist_.push_back(time);
}

void GVINS::parametersStatistic() {

    vector<double> parameters;

    // 所有关键帧
    // All keyframes
    vector<ulong> keyframeids = map_->orderedKeyFrames();
    size_t size               = keyframeids.size();
    if (size < 2) {
        return;
    }
    auto keyframes = map_->keyframes();

    // 最新的关键帧
    // The latest keyframe
    auto frame_cur = keyframes.at(keyframeids[size - 1]);
    auto frame_pre = keyframes.at(keyframeids[size - 2]);

    // 时间戳
    // Time stamp
    parameters.push_back(frame_cur->stamp());
    parameters.push_back(frame_cur->stamp() - frame_pre->stamp());

    // 当前关键帧与上一个关键帧的id差, 即最新关键帧的跟踪帧数
    // Interval
    auto frame_cnt = static_cast<double>(frame_cur->id() - frame_pre->id());
    parameters.push_back(frame_cnt);

    // 特征点数量
    // Feature points
    parameters.push_back(static_cast<double>(frame_cur->numFeatures()));

    // 路标点重投影误差统计
    // Reprojection errors
    vector<double> reprojection_errors;
    for (auto &landmark : map_->landmarks()) {
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), feat->keyPoint()).norm();
            errors.push_back(error);
        }
        if (errors.empty()) {
            LOGE << "Mappoint " << mappoint->id() << " with zero observation";
            continue;
        }
        double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
        reprojection_errors.emplace_back(avg_error);
    }

    double min_error = *std::min_element(reprojection_errors.begin(), reprojection_errors.end());
    parameters.push_back(min_error);
    double max_error = *std::max_element(reprojection_errors.begin(), reprojection_errors.end());
    parameters.push_back(max_error);
    double avg_error = std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0) /
                       static_cast<double>(reprojection_errors.size());
    parameters.push_back(avg_error);
    double sq_sum =
        std::inner_product(reprojection_errors.begin(), reprojection_errors.end(), reprojection_errors.begin(), 0.0);
    double rms_error = std::sqrt(sq_sum / static_cast<double>(reprojection_errors.size()));
    parameters.push_back(rms_error);

    // 迭代次数
    // Iterations
    parameters.push_back(iterations_[0]);
    parameters.push_back(iterations_[1]);

    // 计算耗时
    // Time cost
    parameters.push_back(timecosts_[0]);
    parameters.push_back(timecosts_[1]);
    parameters.push_back(timecosts_[2]);

    // 路标点粗差
    // Outliers
    parameters.push_back(outliers_[0]);
    parameters.push_back(outliers_[1]);

    // 保存数据
    // Dump current parameters
    statfilesaver_->dump(parameters);
    statfilesaver_->flush();
}

bool GVINS::gvinsOutlierCulling() {
    if (map_->keyframes().empty()) {
        return false;
    }

    // 移除非关键帧中的路标点, 不能在遍历中直接移除, 否则破坏了遍历
    // Find outliers first and remove later
    vector<MapPoint::Ptr> mappoints;
    int num_outliers_mappoint = 0;
    int num_outliers_feature  = 0;
    int num1 = 0, num2 = 0, num3 = 0;
    for (auto &landmark : map_->landmarks()) {
        auto mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        // 未参与优化的无效路标点
        // Only those in the sliding window
        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        // 路标点在滑动窗口内的所有观测
        // All the observations for mappoint
        vector<double> errors;
        for (auto &observation : mappoint->observations()) {
            auto feat = observation.lock();
            if (!feat || feat->isOutlier()) {
                continue;
            }
            auto frame = feat->getFrame();
            if (!frame || !frame->isKeyFrame() || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            auto pp = feat->keyPoint();

            // 计算重投影误差
            // Calculate the reprojection error
            double error = camera_->reprojectionError(frame->pose(), mappoint->pos(), pp).norm();

            // 大于3倍阈值, 则禁用当前观测
            // Feature outlier
            if (!tracking_->isGoodToTrack(pp, frame->pose(), mappoint->pos(), 3.0)) {
                feat->setOutlier(true);
                mappoint->decreaseUsedTimes();

                // 如果当前观测帧是路标点的参考帧, 直接设置为outlier
                // Mappoint
                if (frame->id() == mappoint->referenceFrameId()) {
                    mappoint->setOutlier(true);
                    mappoints.push_back(mappoint);
                    num_outliers_mappoint++;
                    num1++;
                    break;
                }
                num_outliers_feature++;
            } else {
                errors.push_back(error);
            }
        }

        // 有效观测不足, 平均重投影误差较大, 则为粗差
        // Mappoint outlier
        if (errors.size() < 2) {
            mappoint->setOutlier(true);
            mappoints.push_back(mappoint);
            num_outliers_mappoint++;
            num2++;
        } else {
            double avg_error = std::accumulate(errors.begin(), errors.end(), 0.0) / static_cast<double>(errors.size());
            if (avg_error > reprojection_error_std_) {
                mappoint->setOutlier(true);
                mappoints.push_back(mappoint);
                num_outliers_mappoint++;
                num3++;
            }
        }
    }

    // 移除outliers
    // Remove the mappoint outliers
    for (auto &mappoint : mappoints) {
        map_->removeMappoint(mappoint);
    }

    LOGI << "Culled " << num_outliers_mappoint << " mappoint with " << num_outliers_feature << " bad observed features "
         << num1 << ", " << num2 << ", " << num3;
    outliers_[0] = num_outliers_mappoint;
    outliers_[1] = num_outliers_feature;

    return true;
}

bool GVINS::gvinsOptimization() {
    static int first_num_iterations  = optimize_num_iterations_ / 4;
    static int second_num_iterations = optimize_num_iterations_ - first_num_iterations;

    TimeCost timecost;

    ceres::Problem::Options problem_options;
    problem_options.enable_fast_removal = true;

    ceres::Problem problem(problem_options);
    ceres::Solver solver;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type         = ceres::DENSE_SCHUR;
    options.max_num_iterations         = first_num_iterations;
    options.num_threads                = 4;

    // 状态参数
    // State parameters
    addStateParameters(problem);

    // 重投影参数
    // Visual parameters
    addReprojectionParameters(problem);

    // 边缘化残差
    // The prior factor
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {
        auto factor = new MarginalizationFactor(last_marginalization_info_);
        problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks_);
    }

    // GNSS残差
    // The GNSS factors
    auto gnss_redisual_block = addGnssFactors(problem, true);

    // 预积分残差
    // The IMU preintegration factors
    addImuFactors(problem);

    // 视觉重投影残差
    // The visual reprojection factors
    auto residual_ids = addReprojectionFactors(problem, true);

    LOGI << "Add " << preintegrationlist_.size() << " preintegration, " << gnsslist_.size() << " GNSS, "
         << residual_ids.size() << " reprojection";

    // 第一次优化
    // The first optimization
    {
        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[0] = summary.num_successful_steps;
        timecosts_[0]  = timecost.costInMillisecond();
    }

    // 粗差检测和剔除
    // Outlier detetion for GNSS and visual
    {
        // Remove factors in the final

        // Do GNSS outlier culling
        gnssOutlierCullingByChi2(problem, gnss_redisual_block);

        // Remove outlier reprojection factors
        removeReprojectionFactorsByChi2(problem, residual_ids, 5.991);

        // Remove all GNSS factors
        for (auto &block : gnss_redisual_block) {
            problem.RemoveResidualBlock(block.first);
        }

        // Add GNSS Factors without loss function
        addGnssFactors(problem, false);
    }

    // 第二次优化
    // The second optimization
    {
        options.max_num_iterations = second_num_iterations;

        timecost.restart();

        solver.Solve(options, &problem, &summary);
        LOGI << summary.BriefReport();

        iterations_[1] = summary.num_successful_steps;
        timecosts_[1]  = timecost.costInMillisecond();

        if (map_->isMaximumKeframes()) {
            // 进行必要的重积分
            // Reintegration during initialization
            doReintegration();
        }
    }

    // 更新参数, 必须的
    // Update the parameters from the optimizer
    updateParametersFromOptimizer();

    // 移除粗差路标点
    // Remove mappoint and feature outliers
    gvinsOutlierCulling();

    return true;
}

void GVINS::gnssOutlierCullingByChi2(ceres::Problem &problem,
                                     vector<std::pair<ceres::ResidualBlockId, GNSS *>> &redisual_block) {
    double chi2_threshold = 7.815;
    double cost, chi2;

    int outliers_counts = 0;
    for (auto &block : redisual_block) {
        auto id    = block.first;
        GNSS *gnss = block.second;

        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
        chi2 = cost * 2;

        if (chi2 > chi2_threshold) {

            // Reweigthed GNSS
            double scale = sqrt(chi2 / chi2_threshold);
            gnss->std *= scale;

            outliers_counts++;
        }
    }

    if (outliers_counts) {
        LOGI << "Detect " << outliers_counts << " GNSS outliers at " << Logging::doubleData(timelist_.back());
    }
}

int GVINS::removeReprojectionFactorsByChi2(ceres::Problem &problem, vector<ceres::ResidualBlockId> &residual_ids,
                                           double chi2) {
    double cost;
    int outlier_features = 0;

    // 进行卡方检验, 判定粗差因子, 待全部判定完成再进行移除, 否则会导致错误
    // Judge first and remove later
    vector<ceres::ResidualBlockId> outlier_residual_ids;
    for (auto &id : residual_ids) {
        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);

        // cost带有1/2系数
        // To chi2
        if (cost * 2.0 > chi2) {
            outlier_features++;
            outlier_residual_ids.push_back(id);
        }
    }

    // 从优化问题中移除所有粗差因子
    // Remove the outliers from the optimizer
    for (auto &id : outlier_residual_ids) {
        problem.RemoveResidualBlock(id);
    }

    LOGI << "Remove " << outlier_features << " reprojection factors";

    return outlier_features;
}

void GVINS::updateParametersFromOptimizer() {
    if (map_->keyframes().empty()) {
        return;
    }

    // 先更新外参, 更新位姿需要外参
    // Update the extrinsic first
    {
        if (optimize_estimate_td_) {
            td_b_c_ = extrinsic_[7];
        }

        if (optimize_estimate_extrinsic_) {
            Pose ext;
            ext.t[0] = extrinsic_[0];
            ext.t[1] = extrinsic_[1];
            ext.t[2] = extrinsic_[2];

            Quaterniond qic = Quaterniond(extrinsic_[6], extrinsic_[3], extrinsic_[4], extrinsic_[5]);
            ext.R           = Rotation::quaternion2matrix(qic.normalized());

            // 外参估计检测, 误差较大则不更新, 1m or 5deg
            double dt = (ext.t - pose_b_c_.t).norm();
            double dr = Rotation::matrix2quaternion(ext.R * pose_b_c_.R.transpose()).vec().norm() * R2D;
            if ((dt > 1.0) || (dr > 5.0)) {
                LOGE << "Estimated extrinsic is too large, t: " << ext.t.transpose()
                     << ", R: " << Rotation::matrix2euler(ext.R).transpose() * R2D;
            } else {
                // Update the extrinsic
                Lock lock(extrinsic_mutex_);
                pose_b_c_ = ext;
            }

            vector<double> extrinsic;
            Vector3d euler = Rotation::matrix2euler(ext.R) * R2D;

            extrinsic.push_back(timelist_.back());
            extrinsic.push_back(ext.t[0]);
            extrinsic.push_back(ext.t[1]);
            extrinsic.push_back(ext.t[2]);
            extrinsic.push_back(euler[0]);
            extrinsic.push_back(euler[1]);
            extrinsic.push_back(euler[2]);
            extrinsic.push_back(td_b_c_);

            extfilesaver_->dump(extrinsic);
            extfilesaver_->flush();
        }
    }

    // 更新关键帧的位姿
    // Update the keyframe pose
    for (auto &keyframe : map_->keyframes()) {
        auto &frame = keyframe.second;
        auto index  = getStateDataIndex(frame->stamp());
        if (index < 0) {
            continue;
        }

        IntegrationState state = Preintegration::stateFromData(statedatalist_[index], preintegration_options_);
        frame->setPose(MISC::stateToCameraPose(state, pose_b_c_));
    }

    // 更新路标点的深度和位置
    // Update the mappoints
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        auto frame = mappoint->referenceFrame();
        if (!frame || !map_->isKeyFrameInMap(frame)) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        double invdepth = invdepthlist_[mappoint->id()];
        double depth    = 1.0 / invdepth;

        auto pc0      = camera_->pixel2cam(mappoint->referenceKeypoint());
        Vector3d pc00 = {pc0.x(), pc0.y(), 1.0};
        pc00 *= depth;

        mappoint->pos() = camera_->cam2world(pc00, mappoint->referenceFrame()->pose());
        mappoint->updateDepth(depth);
    }
}

bool GVINS::gvinsRemoveAllSecondNewFrame() {
    vector<ulong> keyframeids = map_->orderedKeyFrames();

    for (auto id : keyframeids) {
        auto frame = map_->keyframes().find(id)->second;
        // 移除次新帧, 以及倒数第二个空关键帧
        if ((frame->keyFrameState() == KEYFRAME_REMOVE_SECOND_NEW) ||
            (frame->features().empty() && (id != keyframeids.back()))) {
            unused_time_nodes_.push_back(frame->stamp());

            // 仅需要重置关键帧标志, 从地图中移除次新关键帧即可,
            // 无需调整状态参数和路标点
            // Just remove the frame
            frame->resetKeyFrame();
            map_->removeKeyFrame(frame, false);
        }
    }

    return true;
}

bool GVINS::gvinsMarginalization() {

    // 按时间先后排序的关键帧
    // Ordered keyframes
    vector<ulong> keyframeids = map_->orderedKeyFrames();
    auto latest_keyframe      = map_->latestKeyFrame();

    latest_keyframe->setKeyFrameState(KEYFRAME_NORMAL);

    // 对齐到保留的最后一个关键帧, 可能移除多个预积分对象
    // Align to the last keyframe time
    auto frame      = map_->keyframes().find(keyframeids[1])->second;
    size_t num_marg = getStateDataIndex(frame->stamp());

    double last_time = timelist_[num_marg];

    LOGI << "Marginalize " << num_marg << " states, last time " << Logging::doubleData(last_time);

    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();

    // 指定每个参数块独立的ID, 用于索引参数
    // For fixed order
    std::unordered_map<long, long> parameters_ids;
    parameters_ids.clear();
    long parameters_id = 0;

    {
        // 边缘化参数
        // Marginalization parameters
        for (auto &last_marginalization_parameter_block : last_marginalization_parameter_blocks_) {
            parameters_ids[reinterpret_cast<long>(last_marginalization_parameter_block)] = parameters_id++;
        }

        // 外参参数
        // Extrinsic parameters
        parameters_ids[reinterpret_cast<long>(extrinsic_)]     = parameters_id++;
        parameters_ids[reinterpret_cast<long>(extrinsic_ + 7)] = parameters_id++;

        // 位姿参数
        // Pose parameters
        for (const auto &statedata : statedatalist_) {
            parameters_ids[reinterpret_cast<long>(statedata.pose)] = parameters_id++;
            parameters_ids[reinterpret_cast<long>(statedata.mix)]  = parameters_id++;
        }

        // 逆深度参数
        // Inverse depth parameters
        frame         = map_->keyframes().at(keyframeids[0]);
        auto features = frame->features();
        for (auto const &feature : features) {
            auto mappoint = feature.second->getMapPoint();
            if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
                continue;
            }

            if (mappoint->referenceFrame() != frame) {
                continue;
            }

            double *invdepth                                 = &invdepthlist_[mappoint->id()];
            parameters_ids[reinterpret_cast<long>(invdepth)] = parameters_id++;
        }

        // 更新参数块的特定ID, 必要的
        // Update the IS for parameters
        marginalization_info->updateParamtersIds(parameters_ids);
    }

    // 边缘化因子
    // The prior factor
    if (last_marginalization_info_ && last_marginalization_info_->isValid()) {

        std::vector<int> marginalized_index;
        for (size_t i = 0; i < num_marg; i++) {
            for (size_t k = 0; k < last_marginalization_parameter_blocks_.size(); k++) {
                if (last_marginalization_parameter_blocks_[k] == statedatalist_[i].pose ||
                    last_marginalization_parameter_blocks_[k] == statedatalist_[i].mix) {
                    marginalized_index.push_back((int) k);
                }
            }
        }

        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info_);
        auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr, last_marginalization_parameter_blocks_,
                                                            marginalized_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // GNSS因子
    // The GNSS factors
    for (auto &gnss : gnsslist_) {
        for (size_t k = 0; k < num_marg; k++) {
            if (MISC::isTheSameTimeNode(gnss.time, timelist_[k], MISC::MINIMUM_TIME_INTERVAL)) {
                auto factor   = std::make_shared<GnssFactor>(gnss, antlever_);
                auto residual = std::make_shared<ResidualBlockInfo>(
                    factor, nullptr, std::vector<double *>{statedatalist_[k].pose}, std::vector<int>{0});
                marginalization_info->addResidualBlockInfo(residual);
                break;
            }
        }
    }

    // 预积分因子
    // The IMU preintegration factors
    for (size_t k = 0; k < num_marg; k++) {
        // 由于会移除多个预积分, 会导致出现保留和移除同时出现, 判断索引以区分
        // More than one may be removed
        vector<int> marg_index;
        if (k == (num_marg - 1)) {
            marg_index = {0, 1};
        } else {
            marg_index = {0, 1, 2, 3};
        }

        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist_[k]);
        auto residual = std::make_shared<ResidualBlockInfo>(
            factor, nullptr,
            std::vector<double *>{statedatalist_[k].pose, statedatalist_[k].mix, statedatalist_[k + 1].pose,
                                  statedatalist_[k + 1].mix},
            marg_index);
        marginalization_info->addResidualBlockInfo(residual);
    }

    // 重投影因子, 最老的关键帧
    // The visual reprojection factors

    frame         = map_->keyframes().at(keyframeids[0]);
    auto features = frame->features();

    auto loss_function = std::make_shared<ceres::HuberLoss>(1.0);
    for (auto const &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();
        if (ref_frame != frame) {
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());
        size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        if (ref_frame_index < 0) {
            continue;
        }

        double *invdepth = &invdepthlist_[mappoint->id()];

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        auto observations = mappoint->observations();
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();
            if (!obs_frame || !obs_frame->isKeyFrame() || !map_->isKeyFrameInMap(obs_frame) ||
                (obs_frame == ref_frame)) {
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
                LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
                     << Logging::doubleData(obs_frame->stamp());
                continue;
            }

            auto factor = std::make_shared<ReprojectionFactor>(
                ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(), obs_feature->velocityInPixel(),
                ref_frame->timeDelay(), obs_frame->timeDelay(), optimize_reprojection_error_std_);
            auto residual = std::make_shared<ResidualBlockInfo>(factor, nullptr,
                                                                vector<double *>{statedatalist_[ref_frame_index].pose,
                                                                                 statedatalist_[obs_frame_index].pose,
                                                                                 extrinsic_, invdepth, &extrinsic_[7]},
                                                                vector<int>{0, 3});
            marginalization_info->addResidualBlockInfo(residual);
        }
    }

    // 边缘化处理
    // Do marginalization
    marginalization_info->marginalization();

    // 保留的数据, 使用独立ID
    // Update the address
    std::unordered_map<long, double *> address;
    for (size_t k = num_marg; k < statedatalist_.size(); k++) {
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k].pose)]] = statedatalist_[k].pose;
        address[parameters_ids[reinterpret_cast<long>(statedatalist_[k].mix)]]  = statedatalist_[k].mix;
    }
    address[parameters_ids[reinterpret_cast<long>(extrinsic_)]]     = extrinsic_;
    address[parameters_ids[reinterpret_cast<long>(extrinsic_ + 7)]] = &extrinsic_[7];

    last_marginalization_parameter_blocks_ = marginalization_info->getParamterBlocks(address);
    last_marginalization_info_             = std::move(marginalization_info);

    // 移除边缘化的数据
    // Remove the marginalized data

    // GNSS观测
    // The GNSS observations
    size_t num_gnss = gnsslist_.size();
    for (size_t k = 0; k < gnsslist_.size(); k++) {
        if (gnsslist_[k].time > last_time) {
            num_gnss = k;
            break;
        }
    }
    for (size_t k = 0; k < num_gnss; k++) {
        gnsslist_.pop_front();
    }

    // 预积分观测及时间状态
    // The IMU preintegration and time nodes
    for (size_t k = 0; k < num_marg; k++) {
        timelist_.pop_front();
        statedatalist_.pop_front();
        preintegrationlist_.pop_front();
    }

    // 保存移除的路标点, 用于可视化
    // The marginalized mappoints, for visualization
    frame    = map_->keyframes().at(keyframeids[0]);
    features = frame->features();
    for (const auto &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (feature.second->isOutlier() || !mappoint || mappoint->isOutlier()) {
            continue;
        }
        auto &pw = mappoint->pos();

        if (is_use_visualization_) {
            drawer_->addNewFixedMappoint(pw);
        }

        // 保存路标点
        // Save these mappoints to file
        ptsfilesaver_->dump(vector<double>{pw.x(), pw.y(), pw.z()});
    }

    // 关键帧
    // The marginalized keyframe
    map_->removeKeyFrame(frame, true);

    return true;
}

void GVINS::doReintegration() {
    int cnt = 0;
    for (size_t k = 0; k < preintegrationlist_.size(); k++) {
        IntegrationState state = Preintegration::stateFromData(statedatalist_[k], preintegration_options_);
        Vector3d dbg           = preintegrationlist_[k]->deltaState().bg - state.bg;
        Vector3d dba           = preintegrationlist_[k]->deltaState().ba - state.ba;
        if ((dbg.norm() > 6 * integration_parameters_->gyr_bias_std) ||
            (dba.norm() > 6 * integration_parameters_->acc_bias_std)) {
            preintegrationlist_[k]->reintegration(state);
            cnt++;
        }
    }
    if (cnt) {
        LOGW << "Reintegration " << cnt << " preintegration";
    }
}

void GVINS::addReprojectionParameters(ceres::Problem &problem) {
    if (map_->landmarks().empty()) {
        return;
    }

    invdepthlist_.clear();
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            auto frame = mappoint->referenceFrame();
            if (!frame || !map_->isKeyFrameInMap(frame)) {
                continue;
            }

            double depth         = mappoint->depth();
            double inverse_depth = 1.0 / depth;

            // 确保深度数值有效
            // For valid mappoints
            if (std::isnan(inverse_depth)) {
                mappoint->setOutlier(true);
                LOGE << "Mappoint " << mappoint->id() << " is wrong with depth " << depth << " type "
                     << mappoint->mapPointType();
                continue;
            }

            invdepthlist_[mappoint->id()] = inverse_depth;
            problem.AddParameterBlock(&invdepthlist_[mappoint->id()], 1);

            mappoint->addOptimizedTimes();
        }
    }

    // 外参
    // Extrinsic parameters
    extrinsic_[0] = pose_b_c_.t[0];
    extrinsic_[1] = pose_b_c_.t[1];
    extrinsic_[2] = pose_b_c_.t[2];

    Quaterniond qic = Rotation::matrix2quaternion(pose_b_c_.R);
    qic.normalize();
    extrinsic_[3] = qic.x();
    extrinsic_[4] = qic.y();
    extrinsic_[5] = qic.z();
    extrinsic_[6] = qic.w();

    ceres::LocalParameterization *parameterization = new (PoseParameterization);
    problem.AddParameterBlock(extrinsic_, 7, parameterization);

    if (!optimize_estimate_extrinsic_ || gvinsstate_ != GVINS_TRACKING_NORMAL) {
        problem.SetParameterBlockConstant(extrinsic_);
    }

    // 时间延时
    // Time delay
    extrinsic_[7] = td_b_c_;
    problem.AddParameterBlock(&extrinsic_[7], 1);
    if (!optimize_estimate_td_ || gvinsstate_ != GVINS_TRACKING_NORMAL) {
        problem.SetParameterBlockConstant(&extrinsic_[7]);
    }
}

vector<ceres::ResidualBlockId> GVINS::addReprojectionFactors(ceres::Problem &problem, bool isusekernel) {

    vector<ceres::ResidualBlockId> residual_ids;

    if (map_->keyframes().empty()) {
        return residual_ids;
    }

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    residual_ids.clear();
    for (const auto &landmark : map_->landmarks()) {
        const auto &mappoint = landmark.second;
        if (!mappoint || mappoint->isOutlier()) {
            continue;
        }

        if (invdepthlist_.find(mappoint->id()) == invdepthlist_.end()) {
            continue;
        }

        auto ref_frame = mappoint->referenceFrame();
        if (!map_->isKeyFrameInMap(ref_frame)) {
            continue;
        }

        auto ref_frame_pc      = camera_->pixel2cam(mappoint->referenceKeypoint());
        size_t ref_frame_index = getStateDataIndex(ref_frame->stamp());
        if (ref_frame_index < 0) {
            continue;
        }

        double *invdepth = &invdepthlist_[mappoint->id()];
        if (*invdepth == 0) {
            *invdepth = 1.0 / MapPoint::DEFAULT_DEPTH;
        }

        auto ref_feature = ref_frame->features().find(mappoint->id())->second;

        auto observations = mappoint->observations();
        for (auto &observation : observations) {
            auto obs_feature = observation.lock();
            if (!obs_feature || obs_feature->isOutlier()) {
                continue;
            }
            auto obs_frame = obs_feature->getFrame();
            if (!obs_frame || !obs_frame->isKeyFrame() || !map_->isKeyFrameInMap(obs_frame) ||
                (obs_frame == ref_frame)) {
                continue;
            }

            auto obs_frame_pc      = camera_->pixel2cam(obs_feature->keyPoint());
            size_t obs_frame_index = getStateDataIndex(obs_frame->stamp());

            if ((obs_frame_index < 0) || (ref_frame_index == obs_frame_index)) {
                LOGE << "Wrong matched mapoint keyframes " << Logging::doubleData(ref_frame->stamp()) << " with "
                     << Logging::doubleData(obs_frame->stamp());
                continue;
            }

            auto factor = new ReprojectionFactor(ref_frame_pc, obs_frame_pc, ref_feature->velocityInPixel(),
                                                 obs_feature->velocityInPixel(), ref_frame->timeDelay(),
                                                 obs_frame->timeDelay(), optimize_reprojection_error_std_);
            auto residual_block_id =
                problem.AddResidualBlock(factor, loss_function, statedatalist_[ref_frame_index].pose,
                                         statedatalist_[obs_frame_index].pose, extrinsic_, invdepth, &extrinsic_[7]);
            residual_ids.push_back(residual_block_id);
        }
    }

    return residual_ids;
}

int GVINS::getStateDataIndex(double time) {

    size_t index = MISC::getStateDataIndex(timelist_, time, MISC::MINIMUM_TIME_INTERVAL);
    if (!MISC::isTheSameTimeNode(timelist_[index], time, MISC::MINIMUM_TIME_INTERVAL)) {
        LOGE << "Wrong matching time node " << Logging::doubleData(timelist_[index]) << " to "
             << Logging::doubleData(time);
        return -1;
    }
    return static_cast<int>(index);
}

void GVINS::addStateParameters(ceres::Problem &problem) {
    LOGI << "Total " << statedatalist_.size() << " pose states from "
         << Logging::doubleData(statedatalist_.begin()->time) << " to "
         << Logging::doubleData(statedatalist_.back().time);

    for (auto &statedata : statedatalist_) {
        // 位姿
        // Pose
        ceres::LocalParameterization *parameterization = new (PoseParameterization);
        problem.AddParameterBlock(statedata.pose, Preintegration::numPoseParameter(), parameterization);

        // IMU mix parameters
        problem.AddParameterBlock(statedata.mix, Preintegration::numMixParameter(preintegration_options_));
    }
}

void GVINS::addImuFactors(ceres::Problem &problem) {
    for (size_t k = 0; k < preintegrationlist_.size(); k++) {
        // 预积分因子
        // IMU preintegration factors
        auto factor = new PreintegrationFactor(preintegrationlist_[k]);
        problem.AddResidualBlock(factor, nullptr, statedatalist_[k].pose, statedatalist_[k].mix,
                                 statedatalist_[k + 1].pose, statedatalist_[k + 1].mix);
    }

    // 添加IMU误差约束, 限制过大的误差估计
    // IMU error factor
    auto factor = new ImuErrorFactor(preintegrationlist_.back());
    problem.AddResidualBlock(factor, nullptr, statedatalist_[preintegrationlist_.size()].mix);
}

vector<std::pair<ceres::ResidualBlockId, GNSS *>> GVINS::addGnssFactors(ceres::Problem &problem, bool isusekernel) {
    vector<std::pair<ceres::ResidualBlockId, GNSS *>> residual_block;

    ceres::LossFunction *loss_function = nullptr;
    if (isusekernel) {
        loss_function = new ceres::HuberLoss(1.0);
    }

    for (auto &data : gnsslist_) {
        int index = getStateDataIndex(data.time);
        if (index >= 0) {
            auto factor = new GnssFactor(data, antlever_);
            auto id     = problem.AddResidualBlock(factor, loss_function, statedatalist_[index].pose);
            residual_block.push_back(std::make_pair(id, &data));
        }
    }

    return residual_block;
}
