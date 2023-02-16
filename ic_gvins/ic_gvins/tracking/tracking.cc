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

#include "tracking.h"

#include "common/angle.h"
#include "common/logging.h"
#include "common/rotation.h"

#include <tbb/tbb.h>
#include <yaml-cpp/yaml.h>

Tracking::Tracking(Camera::Ptr camera, Map::Ptr map, Drawer::Ptr drawer, const string &configfile,
                   const string &outputpath)
    : frame_cur_(nullptr)
    , frame_ref_(nullptr)
    , camera_(std::move(camera))
    , map_(std::move(map))
    , drawer_(std::move(drawer))
    , isnewkeyframe_(false)
    , isinitializing_(true)
    , histogram_(0) {

    logfilesaver_ = FileSaver::create(outputpath + "/tracking.txt", 3);
    if (!logfilesaver_->isOpen()) {
        LOGE << "Failed to open data file";
        return;
    }

    YAML::Node config;
    std::vector<double> vecdata;
    config = YAML::LoadFile(configfile);

    track_check_histogram_ = config["track_check_histogram"].as<bool>();
    track_min_parallax_    = config["track_min_parallax"].as<double>();
    track_max_features_    = config["track_max_features"].as<int>();
    track_max_interval_    = config["track_max_interval"].as<double>();
    track_max_interval_ *= 0.95; // 错开整时间间隔

    is_use_visualization_   = config["is_use_visualization"].as<bool>();
    reprojection_error_std_ = config["reprojection_error_std"].as<double>();

    // 直方图均衡化
    clahe_ = cv::createCLAHE(3.0, cv::Size(21, 21));

    // 分块索引
    block_cols_ = static_cast<int>(lround(camera_->width() / TRACK_BLOCK_SIZE));
    block_rows_ = static_cast<int>(lround(camera_->height() / TRACK_BLOCK_SIZE));
    block_cnts_ = block_cols_ * block_rows_;

    int col, row;
    row = camera_->height() / block_rows_;
    col = camera_->width() / block_cols_;
    block_indexs_.emplace_back(std::make_pair(col, row));
    for (int i = 0; i < block_rows_; i++) {
        for (int j = 0; j < block_cols_; j++) {
            block_indexs_.emplace_back(std::make_pair(col * j, row * i));
        }
    }

    // 每个分块提取的角点数量
    track_max_block_features_ =
        static_cast<int>(lround(static_cast<double>(track_max_features_) / static_cast<double>(block_cnts_)));

    // 每个格子的提取特征数量平方面积为格子面积的 2/3
    track_min_pixel_distance_ = static_cast<int>(round(TRACK_BLOCK_SIZE / sqrt(track_max_block_features_ * 1.5)));
}

double Tracking::calculateHistigram(const Mat &image) {
    Mat histogram;
    int channels[]         = {0};
    int histsize           = 256;
    float range[]          = {0, 256};
    const float *histrange = {range};
    bool uniform = true, accumulate = false;

    cv::calcHist(&image, 1, channels, Mat(), histogram, 1, &histsize, &histrange, uniform, accumulate);

    double hist = 0;
    for (int k = 0; k < 256; k++) {
        hist += histogram.at<float>(k) * (float) k / 256.0;
    }
    hist /= (image.cols * image.rows);

    return hist;
}

bool Tracking::preprocessing(Frame::Ptr frame) {
    isnewkeyframe_ = false;

    // 彩色转灰度
    if (frame->image().channels() == 3) {
        cv::cvtColor(frame->image(), frame->image(), cv::COLOR_BGR2GRAY);
    }

    if (track_check_histogram_) {
        // 计算直方图参数
        double hist = calculateHistigram(frame->image());
        if (histogram_ != 0) {
            double rate = fabs((hist - histogram_) / histogram_);

            // 图像直方图变化比例大于10%, 则跳过当前帧
            if (rate > 0.1) {
                LOGW << "Histogram change too large at " << Logging::doubleData(frame->stamp()) << " with " << rate;
                passed_cnt_++;

                if (passed_cnt_ > 1) {
                    histogram_ = 0;
                }
                return false;
            }
        }
        histogram_ = hist;
    }

    frame_pre_ = frame_cur_;
    frame_cur_ = std::move(frame);

    // 直方图均衡化
    clahe_->apply(frame_cur_->image(), frame_cur_->image());

    return true;
}

TrackState Tracking::track(Frame::Ptr frame) {
    // Tracking

    timecost_.restart();

    TrackState track_state = TRACK_PASSED;

    // 预处理
    if (!preprocessing(std::move(frame))) {
        return track_state;
    }

    if (isinitializing_) {
        // Initialization
        if (frame_ref_ == nullptr) {
            doResetTracking();

            frame_ref_ = frame_cur_;

            featuresDetection(frame_ref_, false);

            return TRACK_FIRST_FRAME;
        }

        if (pts2d_ref_.empty()) {
            featuresDetection(frame_ref_, false);
        }

        // 从参考帧跟踪过来的特征点
        trackReferenceFrame();

        if (parallax_ref_ < track_min_parallax_) {
            showTracking();
            return TRACK_INITIALIZING;
        }

        LOGI << "Initialization tracking with parallax " << parallax_ref_;

        triangulation();

        if (doResetTracking()) {
            LOGW << "Reset initialization";
            showTracking();

            makeNewFrame(KEYFRAME_NORMAL);
            return TRACK_FIRST_FRAME;
        }

        // 初始化两帧都是关键帧
        frame_ref_->setKeyFrame(KEYFRAME_NORMAL);

        // 新关键帧, 地图更新, 数据转存
        makeNewFrame(KEYFRAME_NORMAL);
        last_keyframe_ = frame_cur_;

        isinitializing_ = false;

        track_state = TRACK_TRACKING;
    } else {
        // Tracking

        // 跟踪上一帧中带路标点的特征, 利用预测的位姿先验
        trackMappoint();

        // 未关联路标点的新特征, 补偿旋转预测
        trackReferenceFrame();

        // 检查关键帧类型
        auto keyframe_state = checkKeyFrameSate();

        // 正常关键帧, 需要三角化路标点
        if ((keyframe_state == KEYFRAME_NORMAL) || (keyframe_state == KEYFRAME_REMOVE_OLDEST)) {
            // 三角化补充路标点
            triangulation();
        } else {
            // 添加新的特征
            featuresDetection(frame_cur_, true);
        }

        // 跟踪失败, 路标点数据严重不足
        if (doResetTracking()) {
            makeNewFrame(KEYFRAME_NORMAL);
            return TRACK_LOST;
        }

        // 观测帧, 进行插入
        if (keyframe_state != KEYFRAME_NONE) {
            makeNewFrame(keyframe_state);
        }

        track_state = TRACK_TRACKING;

        if (keyframe_state != KEYFRAME_NONE) {
            writeLoggingMessage();
        }
    }

    // 显示跟踪情况
    showTracking();

    return track_state;
}

bool Tracking::isGoodDepth(double depth, double scale) {
    return ((depth > MapPoint::NEAREST_DEPTH) && (depth < MapPoint::FARTHEST_DEPTH * scale));
}

void Tracking::makeNewFrame(int state) {
    frame_cur_->setKeyFrame(state);
    isnewkeyframe_ = true;

    // 仅当正常关键帧才更新参考帧
    if ((state == KEYFRAME_NORMAL) || (state == KEYFRAME_REMOVE_OLDEST)) {
        frame_ref_ = frame_cur_;

        featuresDetection(frame_ref_, true);
    }
}

keyFrameState Tracking::checkKeyFrameSate() {
    keyFrameState keyframe_state = KEYFRAME_NONE;

    // 相邻时间太短, 不进行关键帧处理
    double dt = frame_cur_->stamp() - last_keyframe_->stamp();
    if (dt < TRACK_MIN_INTERVAl) {
        return keyframe_state;
    }

    double parallax = (parallax_map_ * parallax_map_counts_ + parallax_ref_ * parallax_ref_counts_) /
                      (parallax_map_counts_ + parallax_ref_counts_);
    if (parallax > track_min_parallax_) {
        // 新的关键帧, 满足最小像素视差

        keyframe_state = map_->isWindowFull() ? KEYFRAME_REMOVE_OLDEST : KEYFRAME_NORMAL;

        LOGI << "Keyframe at " << Logging::doubleData(frame_cur_->stamp()) << ", mappoints "
             << frame_cur_->numFeatures() << ", interval " << dt << ", parallax " << parallax;
    } else if (dt > track_max_interval_) {
        // 普通观测帧, 非关键帧
        keyframe_state = KEYFRAME_REMOVE_SECOND_NEW;
        LOGI << "Keyframe at " << Logging::doubleData(frame_cur_->stamp()) << " due to long interval";
    }

    // 切换上一关键帧, 用于时间间隔计算
    if (keyframe_state != KEYFRAME_NONE) {
        last_keyframe_ = frame_cur_;

        // 更新路标点在观测帧中的使用次数
        for (auto &mappoint : tracked_mappoint_) {
            mappoint->increaseUsedTimes();
        }

        // 输出关键帧信息
        logging_data_.clear();

        logging_data_.push_back(frame_cur_->stamp());
        logging_data_.push_back(dt);
        logging_data_.push_back(parallax);
        logging_data_.push_back(relativeTranslation());
        logging_data_.push_back(relativeRotation());
    }

    return keyframe_state;
}

void Tracking::writeLoggingMessage() {
    logging_data_.push_back(static_cast<double>(frame_cur_->features().size()));
    logging_data_.push_back(timecost_.costInMillisecond());

    logfilesaver_->dump(logging_data_);
    logfilesaver_->flush();
}

bool Tracking::doResetTracking() {
    if (!frame_cur_->numFeatures()) {
        isinitializing_ = true;
        frame_ref_      = frame_cur_;
        pts2d_new_.clear();
        pts2d_ref_.clear();
        pts2d_ref_frame_.clear();
        velocity_ref_.clear();
        return true;
    }

    return false;
}

double Tracking::relativeTranslation() {
    return (frame_cur_->pose().t - frame_ref_->pose().t).norm();
}

double Tracking::relativeRotation() {
    Matrix3d R     = frame_cur_->pose().R.transpose() * frame_ref_->pose().R;
    Vector3d euler = Rotation::matrix2euler(R);

    // Only for heading
    return fabs(euler[1] * R2D);
}

void Tracking::showTracking() {
    if (!is_use_visualization_) {
        return;
    }

    drawer_->updateFrame(frame_cur_);
}

bool Tracking::trackMappoint() {

    // 上一帧中的路标点
    mappoint_matched_.clear();
    vector<cv::Point2f> pts2d_map, pts2d_matched, pts2d_map_undis;
    vector<MapPointType> mappoint_type;
    auto features = frame_pre_->features();
    for (auto &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (mappoint && !mappoint->isOutlier()) {
            mappoint_matched_.push_back(mappoint);
            pts2d_map_undis.push_back(feature.second->keyPoint());
            pts2d_map.push_back(feature.second->distortedKeyPoint());
            mappoint_type.push_back(mappoint->mapPointType());

            // 预测的特征点
            auto pixel = camera_->world2pixel(mappoint->pos(), frame_cur_->pose());

            pts2d_matched.emplace_back(pixel);
        }
    }
    if (pts2d_matched.empty()) {
        LOGE << "No feature with mappoint in previous frame";
        return false;
    }

    // 预测的特征点像素坐标添加畸变, 用于跟踪
    camera_->distortPoints(pts2d_matched);

    vector<uint8_t> status, status_reverse;
    vector<float> error;
    vector<cv::Point2f> pts2d_reverse = pts2d_map;

    // 正向光流
    cv::calcOpticalFlowPyrLK(frame_pre_->image(), frame_cur_->image(), pts2d_map, pts2d_matched, status, error,
                             cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);
    // 反向光流
    cv::calcOpticalFlowPyrLK(frame_cur_->image(), frame_pre_->image(), pts2d_matched, pts2d_reverse, status_reverse,
                             error, cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // 跟踪失败的
    for (size_t k = 0; k < status.size(); k++) {
        if (status[k] && status_reverse[k] && !isOnBorder(pts2d_matched[k]) &&
            (ptsDistance(pts2d_reverse[k], pts2d_map[k]) < 0.5)) {
            status[k] = 1;
        } else {
            status[k] = 0;
        }
    }
    reduceVector(pts2d_map, status);
    reduceVector(pts2d_matched, status);
    reduceVector(mappoint_matched_, status);
    reduceVector(mappoint_type, status);
    reduceVector(pts2d_map_undis, status);

    if (pts2d_matched.empty()) {
        LOGE << "Track previous with mappoint failed";
        // 清除上一帧的跟踪
        if (is_use_visualization_) {
            drawer_->updateTrackedMapPoints({}, {}, {});
        }
        parallax_map_        = 0;
        parallax_map_counts_ = 0;
        return false;
    }

    // 匹配后的点, 需要重新矫正畸变
    auto pts2d_matched_undis = pts2d_matched;
    camera_->undistortPoints(pts2d_matched_undis);

    // 匹配的3D-2D
    frame_cur_->clearFeatures();
    tracked_mappoint_.clear();

    double dt = frame_cur_->stamp() - frame_pre_->stamp();
    for (size_t k = 0; k < pts2d_matched_undis.size(); k++) {
        auto mappoint = mappoint_matched_[k];

        // 将3D-2D匹配到的landmarks指向到当前帧
        auto velocity = (camera_->pixel2cam(pts2d_matched_undis[k]) - camera_->pixel2cam(pts2d_map_undis[k])) / dt;
        auto feature  = Feature::createFeature(frame_cur_, {velocity.x(), velocity.y()}, pts2d_matched_undis[k],
                                               pts2d_matched[k], FEATURE_MATCHED);
        mappoint->addObservation(feature);
        feature->addMapPoint(mappoint);
        frame_cur_->addFeature(mappoint->id(), feature);

        // 用于更新使用次数
        tracked_mappoint_.push_back(mappoint);
    }

    // 路标点跟踪情况
    if (is_use_visualization_) {
        drawer_->updateTrackedMapPoints(pts2d_map, pts2d_matched, mappoint_type);
    }

    parallax_map_counts_ = parallaxFromReferenceMapPoints(parallax_map_);

    LOGI << "Track " << tracked_mappoint_.size() << " map points";

    return true;
}

bool Tracking::trackReferenceFrame() {

    if (pts2d_ref_.empty()) {
        LOGW << "No new feature in previous frame " << Logging::doubleData(frame_cur_->stamp());
        return false;
    }

    // 补偿旋转预测
    Matrix3d r_cur_pre = frame_cur_->pose().R.transpose() * frame_pre_->pose().R;

    // 原始畸变补偿
    auto pts2d_new_undis = pts2d_new_;
    camera_->undistortPoints(pts2d_new_undis);

    pts2d_cur_.clear();
    for (const auto &pp_pre : pts2d_new_undis) {
        Vector3d pc_pre = camera_->pixel2cam(pp_pre);
        Vector3d pc_cur = r_cur_pre * pc_pre;

        // 添加畸变
        auto pp_cur = camera_->distortCameraPoint(pc_cur);
        pts2d_cur_.emplace_back(pp_cur);
    }

    // 跟踪参考帧
    vector<uint8_t> status, status_reverse;
    vector<float> error;
    vector<cv::Point2f> pts2d_reverse = pts2d_new_;

    // 正向光流
    cv::calcOpticalFlowPyrLK(frame_pre_->image(), frame_cur_->image(), pts2d_new_, pts2d_cur_, status, error,
                             cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // 反向光流
    cv::calcOpticalFlowPyrLK(frame_cur_->image(), frame_pre_->image(), pts2d_cur_, pts2d_reverse, status_reverse, error,
                             cv::Size(21, 21), TRACK_PYRAMID_LEVEL,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    // 剔除跟踪失败的, 正向反向跟踪在0.5个像素以内
    for (size_t k = 0; k < status.size(); k++) {
        if (status[k] && status_reverse[k] && !isOnBorder(pts2d_cur_[k]) &&
            (ptsDistance(pts2d_reverse[k], pts2d_new_[k]) < 0.5)) {
            status[k] = 1;
        } else {
            status[k] = 0;
        }
    }
    reduceVector(pts2d_ref_, status);
    reduceVector(pts2d_cur_, status);
    reduceVector(pts2d_new_, status);
    reduceVector(pts2d_ref_frame_, status);
    reduceVector(velocity_ref_, status);

    if (pts2d_ref_.empty()) {
        LOGW << "No new feature in previous frame";
        drawer_->updateTrackedRefPoints({}, {});
        return false;
    }

    // 原始带畸变的角点
    pts2d_new_undis      = pts2d_new_;
    auto pts2d_cur_undis = pts2d_cur_;

    camera_->undistortPoints(pts2d_new_undis);
    camera_->undistortPoints(pts2d_cur_undis);

    // 计算像素速度
    velocity_cur_.clear();
    double dt = frame_cur_->stamp() - frame_pre_->stamp();

    for (size_t k = 0; k < pts2d_cur_undis.size(); k++) {
        Vector3d vel      = (camera_->pixel2cam(pts2d_cur_undis[k]) - camera_->pixel2cam(pts2d_new_undis[k])) / dt;
        Vector2d velocity = {vel.x(), vel.y()};
        velocity_cur_.push_back(velocity);

        // 在关键帧后新增加的特征
        if (pts2d_ref_frame_[k]->id() > frame_ref_->id()) {
            velocity_ref_[k] = velocity;
        }
    }

    // 计算视差
    auto pts2d_ref_undis = pts2d_ref_;
    camera_->undistortPoints(pts2d_ref_undis);
    parallax_ref_counts_ = parallaxFromReferenceKeyPoints(pts2d_ref_undis, pts2d_cur_undis, parallax_ref_);

    // Fundamental粗差剔除
    if (pts2d_cur_.size() >= 15) {
        cv::findFundamentalMat(pts2d_new_undis, pts2d_cur_undis, cv::FM_RANSAC, reprojection_error_std_, 0.99, status);

        reduceVector(pts2d_ref_, status);
        reduceVector(pts2d_cur_, status);
        reduceVector(pts2d_ref_frame_, status);
        reduceVector(velocity_cur_, status);
        reduceVector(velocity_ref_, status);
    }

    if (pts2d_cur_.empty()) {
        LOGW << "No new feature in previous frame";
        drawer_->updateTrackedRefPoints({}, {});
        return false;
    }

    // 从参考帧跟踪过来的新特征点
    if (is_use_visualization_) {
        drawer_->updateTrackedRefPoints(pts2d_ref_, pts2d_cur_);
    }

    // 用于下一帧的跟踪
    pts2d_new_ = pts2d_cur_;

    LOGI << "Track " << pts2d_new_.size() << " reference points";

    return !pts2d_new_.empty();
}

void Tracking::featuresDetection(Frame::Ptr &frame, bool ismask) {

    // 特征点足够则无需提取
    int num_features = static_cast<int>(frame->features().size() + pts2d_ref_.size());
    if (num_features > (track_max_features_ - 5)) {
        return;
    }

    // 初始化分配内存
    int features_cnts[block_cnts_];
    vector<vector<cv::Point2f>> block_features(block_cnts_);
    // 必要的分配内存, 否则并行会造成数据结构错乱
    for (auto &block : block_features) {
        block.reserve(track_max_block_features_);
    }
    for (int k = 0; k < block_cnts_; k++) {
        features_cnts[k] = 0;
    }

    // 计算每个分块已有特征点数量
    int col, row;
    for (const auto &feature : frame->features()) {
        col = int(feature.second->keyPoint().x / (float) block_indexs_[0].first);
        row = int(feature.second->keyPoint().y / (float) block_indexs_[0].second);
        features_cnts[row * block_cols_ + col]++;
    }
    for (auto &pts2d : pts2d_new_) {
        col = int(pts2d.x / (float) block_indexs_[0].first);
        row = int(pts2d.y / (float) block_indexs_[0].second);
        features_cnts[row * block_cols_ + col]++;
    }

    // 设置感兴趣区域, 没有特征的区域
    Mat mask = Mat(camera_->size(), CV_8UC1, 255);
    if (ismask) {
        // 已经跟踪上的点
        for (const auto &pt : frame_cur_->features()) {
            cv::circle(mask, pt.second->keyPoint(), track_min_pixel_distance_, 0, cv::FILLED);
        }

        // 还在跟踪的点
        for (const auto &pts2d : pts2d_new_) {
            cv::circle(mask, pts2d, track_min_pixel_distance_, 0, cv::FILLED);
        }
    }

    // 亚像素角点提取参数
    cv::Size win_size          = cv::Size(5, 5);
    cv::Size zero_zone         = cv::Size(-1, -1);
    cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.01);

    auto tracking_function = [&](const tbb::blocked_range<int> &range) {
        for (int k = range.begin(); k != range.end(); k++) {
            int blocl_track_num = track_max_block_features_ - features_cnts[k];
            if (blocl_track_num > 0) {

                int cols = k % block_cols_;
                int rows = k / block_cols_;

                int col_sta = cols * block_indexs_[0].first;
                int col_end = col_sta + block_indexs_[0].first;
                int row_sta = rows * block_indexs_[0].second;
                int row_end = row_sta + block_indexs_[0].second;
                if (k != (block_cnts_ - 1)) {
                    col_end -= 5;
                    row_end -= 5;
                }

                Mat block_image = frame->image().colRange(col_sta, col_end).rowRange(row_sta, row_end);
                Mat block_mask  = mask.colRange(col_sta, col_end).rowRange(row_sta, row_end);

                cv::goodFeaturesToTrack(block_image, block_features[k], blocl_track_num, 0.01,
                                        track_min_pixel_distance_, block_mask);
                if (!block_features[k].empty()) {
                    // 获取亚像素角点
                    cv::cornerSubPix(block_image, block_features[k], win_size, zero_zone, term_crit);
                }
            }
        }
    };
    tbb::parallel_for(tbb::blocked_range<int>(0, block_cnts_), tracking_function);

    // 调整角点的坐标
    int num_new_features = 0;

    // 连续跟踪的角点, 未三角化的点
    if (!ismask) {
        pts2d_new_.clear();
        pts2d_ref_.clear();
        pts2d_ref_frame_.clear();
        velocity_ref_.clear();
    }

    for (int k = 0; k < block_cnts_; k++) {
        col = k % block_cols_;
        row = k / block_cols_;

        for (const auto &point : block_features[k]) {
            float x = static_cast<float>(col * block_indexs_[0].first) + point.x;
            float y = static_cast<float>(row * block_indexs_[0].second) + point.y;

            auto pts2d = cv::Point2f(x, y);
            pts2d_ref_.push_back(pts2d);
            pts2d_new_.push_back(pts2d);
            pts2d_ref_frame_.push_back(frame);
            velocity_ref_.emplace_back(0, 0);

            num_new_features++;
        }
    }

    LOGI << "Add " << num_new_features << " new features to " << num_features;
}

bool Tracking::triangulation() {
    // 无跟踪上的特征
    if (pts2d_cur_.empty()) {
        return false;
    }

    Pose pose0;
    Pose pose1 = frame_cur_->pose();

    Eigen::Matrix<double, 3, 4> T_c_w_0, T_c_w_1;
    T_c_w_1 = pose2Tcw(pose1).topRows<3>();

    int num_succeeded = 0;
    int num_outlier   = 0;
    int num_reset     = 0;
    int num_outtime   = 0;

    // 原始带畸变的角点
    auto pts2d_ref_undis = pts2d_ref_;
    auto pts2d_cur_undis = pts2d_cur_;

    // 矫正畸变以进行三角化
    camera_->undistortPoints(pts2d_ref_undis);
    camera_->undistortPoints(pts2d_cur_undis);

    // 计算使用齐次坐标, 相机坐标系
    vector<uint8_t> status;
    for (size_t k = 0; k < pts2d_cur_.size(); k++) {
        auto pp0 = pts2d_ref_undis[k];
        auto pp1 = pts2d_cur_undis[k];

        // 参考帧
        auto frame_ref = pts2d_ref_frame_[k];
        if (frame_ref->id() > frame_ref_->id()) {
            // 中途添加的特征, 修改参考帧
            pts2d_ref_frame_[k] = frame_cur_;
            pts2d_ref_[k]       = pts2d_cur_[k];
            status.push_back(1);
            num_reset++;
            continue;
        }

        // 移除长时间跟踪导致参考帧已经不在窗口内的观测
        if (map_->isWindowNormal() && !map_->isKeyFrameInMap(frame_ref)) {
            status.push_back(0);
            num_outtime++;
            continue;
        }

        // 进行必要的视差检查, 保证三角化有效
        pose0           = frame_ref->pose();
        double parallax = keyPointParallax(pts2d_ref_undis[k], pts2d_cur_undis[k], pose0, pose1);
        if (parallax < TRACK_MIN_PARALLAX) {
            status.push_back(1);
            continue;
        }

        T_c_w_0 = pose2Tcw(pose0).topRows<3>();

        // 三角化
        Vector3d pc0 = camera_->pixel2cam(pts2d_ref_undis[k]);
        Vector3d pc1 = camera_->pixel2cam(pts2d_cur_undis[k]);
        Vector3d pw;
        triangulatePoint(T_c_w_0, T_c_w_1, pc0, pc1, pw);

        // 三角化错误的点剔除
        if (!isGoodToTrack(pp0, pose0, pw, 1.0, 3.0) || !isGoodToTrack(pp1, pose1, pw, 1.0, 3.0)) {
            status.push_back(0);
            num_outlier++;
            continue;
        }
        status.push_back(0);
        num_succeeded++;

        // 新的路标点, 加入新的观测, 路标点加入地图
        auto pc       = camera_->world2cam(pw, frame_ref->pose());
        double depth  = pc.z();
        auto mappoint = MapPoint::createMapPoint(frame_ref, pw, pts2d_ref_undis[k], depth, MAPPOINT_TRIANGULATED);

        auto feature = Feature::createFeature(frame_cur_, velocity_cur_[k], pts2d_cur_undis[k], pts2d_cur_[k],
                                              FEATURE_TRIANGULATED);
        mappoint->addObservation(feature);
        feature->addMapPoint(mappoint);
        frame_cur_->addFeature(mappoint->id(), feature);
        mappoint->increaseUsedTimes();

        feature = Feature::createFeature(frame_ref, velocity_ref_[k], pts2d_ref_undis[k], pts2d_ref_[k],
                                         FEATURE_TRIANGULATED);
        mappoint->addObservation(feature);
        feature->addMapPoint(mappoint);
        frame_ref->addFeature(mappoint->id(), feature);
        mappoint->increaseUsedTimes();

        // 新三角化的路标点缓存到最新的关键帧, 不直接加入地图
        frame_cur_->addNewUnupdatedMappoint(mappoint);
    }

    // 由于视差不够未及时三角化的角点
    reduceVector(pts2d_ref_, status);
    reduceVector(pts2d_ref_frame_, status);
    reduceVector(pts2d_cur_, status);
    reduceVector(velocity_ref_, status);

    pts2d_new_ = pts2d_cur_;

    LOGI << "Triangulate " << num_succeeded << " 3D points with " << pts2d_cur_.size() << " left, " << num_reset
         << " reset, " << num_outtime << " outtime and " << num_outlier << " outliers";
    return true;
}

void Tracking::triangulatePoint(const Eigen::Matrix<double, 3, 4> &pose0, const Eigen::Matrix<double, 3, 4> &pose1,
                                const Eigen::Vector3d &pc0, const Eigen::Vector3d &pc1, Eigen::Vector3d &pw) {
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();

    design_matrix.row(0) = pc0[0] * pose0.row(2) - pose0.row(0);
    design_matrix.row(1) = pc0[1] * pose0.row(2) - pose0.row(1);
    design_matrix.row(2) = pc1[0] * pose1.row(2) - pose1.row(0);
    design_matrix.row(3) = pc1[1] * pose1.row(2) - pose1.row(1);

    Eigen::Vector4d point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    pw                    = point.head<3>() / point(3);
}

bool Tracking::isGoodToTrack(const cv::Point2f &pp, const Pose &pose, const Vector3d &pw, double scale,
                             double depth_scale) {
    // 当前相机坐标系
    Vector3d pc = camera_->world2cam(pw, pose);

    // 深度检查
    if (!isGoodDepth(pc[2], depth_scale)) {
        return false;
    }

    // 重投影误差检查
    if (camera_->reprojectionError(pose, pw, pp).norm() > reprojection_error_std_ * scale) {
        return false;
    }

    return true;
}

template <typename T> void Tracking::reduceVector(T &vec, vector<uint8_t> status) {
    size_t index = 0;
    for (size_t k = 0; k < vec.size(); k++) {
        if (status[k]) {
            vec[index++] = vec[k];
        }
    }
    vec.resize(index);
}

double Tracking::ptsDistance(cv::Point2f &pt1, cv::Point2f &pt2) {
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

bool Tracking::isOnBorder(const cv::Point2f &pts) {
    return pts.x < 5.0 || pts.y < 5.0 || (pts.x > (camera_->width() - 5.0)) || (pts.y > (camera_->height() - 5.0));
}

Eigen::Matrix4d Tracking::pose2Tcw(const Pose &pose) {
    Eigen::Matrix4d Tcw;
    Tcw.setZero();
    Tcw(3, 3) = 1;

    Tcw.block<3, 3>(0, 0) = pose.R.transpose();
    Tcw.block<3, 1>(0, 3) = -pose.R.transpose() * pose.t;
    return Tcw;
}

double Tracking::keyPointParallax(const cv::Point2f &pp0, const cv::Point2f &pp1, const Pose &pose0,
                                  const Pose &pose1) {
    Vector3d pc0 = camera_->pixel2cam(pp0);
    Vector3d pc1 = camera_->pixel2cam(pp1);

    // 补偿掉旋转
    Vector3d pc01 = pose1.R.transpose() * pose0.R * pc0;

    // 像素大小
    return (pc01.head<2>() - pc1.head<2>()).norm() * camera_->focalLength();
}

int Tracking::parallaxFromReferenceMapPoints(double &parallax) {

    parallax      = 0;
    int counts    = 0;
    auto features = frame_ref_->features();

    for (auto &feature : features) {
        auto mappoint = feature.second->getMapPoint();
        if (mappoint && !mappoint->isOutlier()) {
            // 取最新的一个路标点观测
            auto observations = mappoint->observations();
            if (observations.empty()) {
                continue;
            }
            auto feat = observations.back().lock();
            if (feat && !feat->isOutlier()) {
                auto frame = feat->getFrame();
                if (frame && (frame == frame_cur_)) {
                    // 对应同一路标点在当前帧的像素观测
                    parallax += keyPointParallax(feature.second->keyPoint(), feat->keyPoint(), frame_ref_->pose(),
                                                 frame_cur_->pose());
                    counts++;
                }
            }
        }
    }

    if (counts != 0) {
        parallax /= counts;
    }

    return counts;
}

int Tracking::parallaxFromReferenceKeyPoints(const vector<cv::Point2f> &ref, const vector<cv::Point2f> &cur,
                                             double &parallax) {
    parallax   = 0;
    int counts = 0;
    for (size_t k = 0; k < pts2d_ref_frame_.size(); k++) {
        if (pts2d_ref_frame_[k] == frame_ref_) {
            parallax += keyPointParallax(ref[k], cur[k], frame_ref_->pose(), frame_cur_->pose());
            counts++;
        }
    }
    if (counts != 0) {
        parallax /= counts;
    }

    return counts;
}
