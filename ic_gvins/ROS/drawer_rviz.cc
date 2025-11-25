/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 */

 #include "ROS/drawer_rviz.h"

 #include "common/logging.h"
 #include "common/rotation.h"
 
 #include <nav_msgs/Odometry.h>
 #include <sensor_msgs/Image.h>
 #include <sensor_msgs/PointCloud.h>
 #include <sensor_msgs/image_encodings.h>
 
 DrawerRviz::DrawerRviz(ros::NodeHandle &nh)
     : isfinished_(false)
     , isframerdy_(false)
     , ismaprdy_(false) {
 
     frame_id_ = "world";
 
     pose_pub_           = nh.advertise<nav_msgs::Odometry>("pose", 2);
     path_pub_           = nh.advertise<nav_msgs::Path>("path", 2);
     gnss_path_pub_      = nh.advertise<nav_msgs::Path>("gnss_path", 2);
     track_image_pub_    = nh.advertise<sensor_msgs::Image>("tracking", 2);
     fixed_points_pub_   = nh.advertise<sensor_msgs::PointCloud>("fixed", 2);
     current_points_pub_ = nh.advertise<sensor_msgs::PointCloud>("current", 2);
 
     // ★★★ 新增：訂閱 YOLO 遮罩 ★★★
     yolo_mask_sub_ = nh.subscribe("/yolo/mask", 10, &DrawerRviz::yoloMaskCallback, this);
 
     LOGI << "DrawerRviz: Subscribed to /yolo/mask for visualization";
     LOGI << "DrawerRviz: Publishing GPS path on topic /gnss_path";
 }
 
 // ★★★ 新增：YOLO 遮罩回調函數 ★★★
 void DrawerRviz::yoloMaskCallback(const sensor_msgs::ImageConstPtr& mask_msg) {
     try {
         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(mask_msg, "mono8");
         
         std::lock_guard<std::mutex> lock(mask_mutex_);
         yolo_mask_ = cv_ptr->image.clone();
         
     } catch (cv_bridge::Exception& e) {
         LOGE << "DrawerRviz: CV Bridge exception: " << e.what();
     }
 }
 
 void DrawerRviz::setFinished() {
     isfinished_ = true;
     update_sem_.notify_one();
 }
 
 void DrawerRviz::run() {
 
     while (!isfinished_) {
         // 等待繪圖更新信號
         std::unique_lock<std::mutex> lock(update_mutex_);
         update_sem_.wait(lock);
 
         // 發布跟蹤的圖像
         if (isframerdy_) {
             publishTrackingImage();
 
             isframerdy_ = false;
         }
 
         // 發布軌跡和地圖點
         if (ismaprdy_) {
             publishOdometry();
 
             publishMapPoints();
 
             ismaprdy_ = false;
         }
     }
 }
 
 void DrawerRviz::updateFrame(Frame::Ptr frame) {
     std::unique_lock<std::mutex> lock(image_mutex_);
 
     frame->image().copyTo(raw_image_);
 
     isframerdy_ = true;
     update_sem_.notify_one();
 }
 
 void DrawerRviz::updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                         vector<MapPointType> mappoint_type) {
     std::unique_lock<std::mutex> lock(image_mutex_);
     pts2d_map_     = std::move(map);
     pts2d_matched_ = std::move(matched);
     mappoint_type_ = std::move(mappoint_type);
 }
 
 void DrawerRviz::updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) {
     std::unique_lock<std::mutex> lock(image_mutex_);
     pts2d_ref_ = std::move(ref);
     pts2d_cur_ = std::move(cur);
 }
 
 void DrawerRviz::publishTrackingImage() {
     std::unique_lock<std::mutex> lock(image_mutex_);
 
     Mat drawed;
     drawTrackingImage(raw_image_, drawed);
     
     // ★★★ 新增：疊加 YOLO 遮罩 ★★★
     {
         std::lock_guard<std::mutex> mask_lock(mask_mutex_);
         
         if (!yolo_mask_.empty() && yolo_mask_.size() == drawed.size()) {
             // 創建綠色半透明遮罩層
             cv::Mat green_overlay = cv::Mat::zeros(drawed.size(), drawed.type());
             
             // 找到動態物件區域（黑色 = 0）
             cv::Mat dynamic_region = (yolo_mask_ == 0);
             
             // 將動態區域塗成綠色
             green_overlay.setTo(cv::Scalar(0, 255, 0), dynamic_region);
             
             // 半透明疊加（alpha = 0.4，即 40% 透明度）
             double alpha = 0.4;
             cv::addWeighted(drawed, 1.0, green_overlay, alpha, 0, drawed);
             
             // 可選：在圖像上添加文字說明
             cv::putText(drawed, "Green = Dynamic Objects (YOLO)", 
                        cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 
                        0.6, 
                        cv::Scalar(0, 255, 0), 
                        2);
         }
     }
 
     sensor_msgs::Image image;
 
     image.header.stamp    = ros::Time::now();
     image.header.frame_id = frame_id_;
     image.encoding        = sensor_msgs::image_encodings::BGR8;
     image.height          = drawed.rows;
     image.width           = drawed.cols;
 
     size_t size = image.height * image.width * 3;
     image.step  = image.width * 3;
     image.data.resize(size);
     memcpy(image.data.data(), drawed.data, size);
 
     track_image_pub_.publish(image);
 }
 
 void DrawerRviz::publishMapPoints() {
     std::unique_lock<std::mutex> lock(map_mutex_);
 
     auto stamp = ros::Time::now();
 
     // 發布窗口內的路標點
     sensor_msgs::PointCloud current_pointcloud;
 
     current_pointcloud.header.stamp    = stamp;
     current_pointcloud.header.frame_id = frame_id_;
 
     // 獲取當前點雲
     for (const auto &local : map_->landmarks()) {
         if (local.second && !local.second->isOutlier()) {
             geometry_msgs::Point32 point;
             point.x = static_cast<float>(local.second->pos().x());
             point.y = static_cast<float>(local.second->pos().y());
             point.z = static_cast<float>(local.second->pos().z());
 
             current_pointcloud.points.push_back(point);
         }
     }
     current_points_pub_.publish(current_pointcloud);
 
     // 發布新的地圖點
     sensor_msgs::PointCloud fixed_pointcloud;
 
     fixed_pointcloud.header.stamp    = stamp;
     fixed_pointcloud.header.frame_id = frame_id_;
 
     for (const auto &pts : fixed_mappoints_) {
         geometry_msgs::Point32 point;
         point.x = static_cast<float>(pts.x());
         point.y = static_cast<float>(pts.y());
         point.z = static_cast<float>(pts.z());
 
         fixed_pointcloud.points.push_back(point);
     }
     fixed_points_pub_.publish(fixed_pointcloud);
 
     fixed_mappoints_.clear();
 }
 
 void DrawerRviz::publishOdometry() {
     std::unique_lock<std::mutex> lock(map_mutex_);
 
     nav_msgs::Odometry odometry;
 
     auto quaternion = Rotation::matrix2quaternion(pose_.R);
     auto stamp      = ros::Time::now();
 
     // Odometry
     odometry.header.stamp            = stamp;
     odometry.header.frame_id         = frame_id_;
     odometry.pose.pose.position.x    = pose_.t.x();
     odometry.pose.pose.position.y    = pose_.t.y();
     odometry.pose.pose.position.z    = pose_.t.z();
     odometry.pose.pose.orientation.x = quaternion.x();
     odometry.pose.pose.orientation.y = quaternion.y();
     odometry.pose.pose.orientation.z = quaternion.z();
     odometry.pose.pose.orientation.w = quaternion.w();
     pose_pub_.publish(odometry);
 
     // Path
     geometry_msgs::PoseStamped pose_stamped;
     pose_stamped.header.stamp    = stamp;
     pose_stamped.header.frame_id = frame_id_;
     pose_stamped.pose            = odometry.pose.pose;
 
     path_.header.stamp    = stamp;
     path_.header.frame_id = frame_id_;
     path_.poses.push_back(pose_stamped);
     path_pub_.publish(path_);
 }
 
 void DrawerRviz::addNewFixedMappoint(Vector3d point) {
     std::unique_lock<std::mutex> lock(map_mutex_);
 
     fixed_mappoints_.push_back(point);
 }
 
 void DrawerRviz::updateMap(const Eigen::Matrix4d &pose) {
     std::unique_lock<std::mutex> lock(map_mutex_);
 
     pose_.R = pose.block<3, 3>(0, 0);
     pose_.t = pose.block<3, 1>(0, 3);
 
     ismaprdy_ = true;
     update_sem_.notify_one();
 }
 
 void DrawerRviz::updateGnssPosition(const Eigen::Vector3d &enu) {
     std::unique_lock<std::mutex> lock(map_mutex_);
 
     // 注意：传入的坐标已经是ENU坐标（在ic_gvins.cc中已经转换过了）
     // 直接使用即可，无需再次转换
 
     // 创建GPS路径点
     auto stamp = ros::Time::now();
     geometry_msgs::PoseStamped gnss_pose;
     gnss_pose.header.stamp    = stamp;
     gnss_pose.header.frame_id = frame_id_;
     gnss_pose.pose.position.x = enu.x();
     gnss_pose.pose.position.y = enu.y();
     gnss_pose.pose.position.z = enu.z();
     gnss_pose.pose.orientation.w = 1.0;  // 无旋转信息
 
     // 添加到GPS路径
     gnss_path_.header.stamp    = stamp;
     gnss_path_.header.frame_id = frame_id_;
     gnss_path_.poses.push_back(gnss_pose);
 
     // 发布GPS路径
     gnss_path_pub_.publish(gnss_path_);
 }