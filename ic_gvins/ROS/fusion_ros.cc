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

 #include "fusion_ros.h"
 #include "drawer_rviz.h"
 
 #include "ic_gvins/common/angle.h"
 #include "ic_gvins/common/gpstime.h"
 #include "ic_gvins/common/logging.h"
 #include "ic_gvins/misc.h"
 #include "ic_gvins/tracking/frame.h"
 
 #include <sstream>
 #include <vector>
 
 #include <yaml-cpp/yaml.h>
 
 #include <boost/filesystem.hpp>
 #include <sensor_msgs/image_encodings.h>
 
 #include <atomic>
 #include <csignal>
 #include <memory>
 
 std::atomic<bool> isfinished{false};
 
 void sigintHandler(int sig);
 void checkStateThread(std::shared_ptr<FusionROS> fusion);
 
 void FusionROS::setFinished() {
     if (gvins_ && gvins_->isRunning()) {
         gvins_->setFinished();
     }
 }
 
 void FusionROS::run() {
     ros::NodeHandle nh;
     ros::NodeHandle pnh("~");
 
     string imu_topic, gnss_topic, image_topic, livox_topic;
     pnh.param<string>("imu_topic", imu_topic, "/imu0");
     pnh.param<string>("gnss_topic", gnss_topic, "/gnss0");
     pnh.param<string>("image_topic", image_topic, "/cam0");
 
     string configfile;
     pnh.param<string>("configfile", configfile, "gvins.yaml");
 
     YAML::Node config;
     std::vector<double> vecdata;
     try {
         config = YAML::LoadFile(configfile);
     } catch (YAML::Exception &exception) {
         std::cout << "Failed to open configuration file" << std::endl;
         return;
     }
     auto outputpath        = config["outputpath"].as<string>();
     auto is_make_outputdir = config["is_make_outputdir"].as<bool>();
 
     if (!boost::filesystem::is_directory(outputpath)) {
         boost::filesystem::create_directory(outputpath);
     }
     if (!boost::filesystem::is_directory(outputpath)) {
         std::cout << "Failed to open outputpath" << std::endl;
         return;
     }
 
     if (is_make_outputdir) {
         absl::CivilSecond cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
         absl::StrAppendFormat(&outputpath, "/T%04d%02d%02d%02d%02d%02d", cs.year(), cs.month(), cs.day(), cs.hour(),
                               cs.minute(), cs.second());
         boost::filesystem::create_directory(outputpath);
     }
 
     isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
     gnssoutagetime_  = config["gnssoutagetime"].as<double>();
     gnssthreshold_   = config["gnssthreshold"].as<double>();
 
     FLAGS_log_dir = outputpath;
 
     Drawer::Ptr drawer = std::make_shared<DrawerRviz>(nh);
     gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);
 
     if (!gvins_->isRunning()) {
         LOGE << "Fusion ROS terminate";
         return;
     }
 
     ros::Subscriber imu_sub   = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &FusionROS::imuCallback, this);
     ros::Subscriber gnss_sub  = nh.subscribe<sensor_msgs::NavSatFix>(gnss_topic, 1, &FusionROS::gnssCallback, this);
     ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(image_topic, 20, &FusionROS::imageCallback, this);
 
     // [新增] 订阅 YOLO mask 用于动态物体过滤
     ros::Subscriber yolo_mask_sub = nh.subscribe<sensor_msgs::Image>("/yolo/mask", 10, &FusionROS::yoloMaskCallback, this);
     LOGI << "FusionROS: Subscribed to /yolo/mask for dynamic feature filtering";
 
     ros::spin();
 }
 
 void FusionROS::imuCallback(const sensor_msgs::ImuConstPtr &imumsg) {
     imu_pre_ = imu_;
 
     double unixsecond = imumsg->header.stamp.toSec();
     double weeksec;
     int week;
     GpsTime::unix2gps(unixsecond, week, weeksec);
 
     imu_.time = weeksec;
     imu_.dt = imu_.time - imu_pre_.time;
 
     imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
     imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
     imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
     imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
     imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
     imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;
 
     if (imu_pre_.time == 0) {
         return;
     }
 
     imu_buffer_.push(imu_);
     while (!imu_buffer_.empty()) {
         auto imu = imu_buffer_.front();
         if (gvins_->addNewImu(imu)) {
             imu_buffer_.pop();
         } else {
             break;
         }
     }
 }
 
void FusionROS::gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg) {
    // Time convertion
    double unixsecond = gnssmsg->header.stamp.toSec();
    double weeksec;
    int week;
    GpsTime::unix2gps(unixsecond, week, weeksec);

    gnss_.time = weeksec;

    gnss_.blh[0] = gnssmsg->latitude * D2R;
    gnss_.blh[1] = gnssmsg->longitude * D2R;
    gnss_.blh[2] = gnssmsg->altitude;
    gnss_.std[0] = sqrt(gnssmsg->position_covariance[4]); // N
    gnss_.std[1] = sqrt(gnssmsg->position_covariance[0]); // E
    gnss_.std[2] = sqrt(gnssmsg->position_covariance[8]); // D

    gnss_.isyawvalid = false;

    // Exception
    if ((gnss_.std[0] == 0) || (gnss_.std[1] == 0) || (gnss_.std[2] == 0)) {
        return;
    }

    // [GPS Visualization] Update GPS path before threshold check to show ALL GPS in RViz
    // This helps diagnose GPS quality issues by showing both accepted and rejected GPS
    gvins_->updateGnssVisualizationOnly(gnss_);

    // Remove bad GNSS
    bool isoutage = false;
    if ((gnss_.std[0] < gnssthreshold_) && (gnss_.std[1] < gnssthreshold_) && (gnss_.std[2] < gnssthreshold_)) {

        if (isusegnssoutage_ && (weeksec >= gnssoutagetime_)) {
            isoutage = true;
        }

        // add new GNSS to GVINS
        if (!isoutage) {
            gvins_->addNewGnss(gnss_);
        }
    }
}

 
 void FusionROS::imageCallback(const sensor_msgs::ImageConstPtr &imagemsg) {
     Mat image;
 
     if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
         image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
         memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
     } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
         image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
         memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
     }
 
     double unixsecond = imagemsg->header.stamp.toSec();
     double weeksec;
     int week;
     GpsTime::unix2gps(unixsecond, week, weeksec);
 
     frame_ = Frame::createFrame(weeksec, image);
 
     frame_buffer_.push(frame_);
     while (!frame_buffer_.empty()) {
         auto frame = frame_buffer_.front();
         if (gvins_->addNewFrame(frame)) {
             frame_buffer_.pop();
         } else {
             break;
         }
     }
 }
 
 // [新增] YOLO mask 回调函数
 void FusionROS::yoloMaskCallback(const sensor_msgs::ImageConstPtr &mask_msg) {
     // 将 YOLO mask 传递给 Tracking 模块进行特征过滤
     if (gvins_ && gvins_->isRunning()) {
         auto tracking = gvins_->getTracking();
         if (tracking) {
             tracking->yoloMaskCallback(mask_msg);
         }
     }
 }
 
 void sigintHandler(int sig) {
     std::cout << "Terminate by Ctrl+C " << sig << std::endl;
     isfinished = true;
 }
 
 void checkStateThread(std::shared_ptr<FusionROS> fusion) {
     std::cout << "Check thread is started..." << std::endl;
 
     auto fusion_ptr = std::move(fusion);
     while (!isfinished) {
         sleep(1);
     }
 
     fusion_ptr->setFinished();
 
     std::cout << "GVINS has been shutdown ..." << std::endl;
 
     ros::shutdown();
 
     std::cout << "ROS node has been shutdown ..." << std::endl;
 }
 
 int main(int argc, char *argv[]) {
     Logging::initialization(argv, true, true);
 
     ros::init(argc, argv, "gvins_node", ros::init_options::NoSigintHandler);
 
     std::signal(SIGINT, sigintHandler);
 
     auto fusion = std::make_shared<FusionROS>();
 
     std::thread check_thread(checkStateThread, fusion);
 
     std::cout << "Fusion process is started..." << std::endl;
 
     fusion->run();
 
     return 0;
 }