/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 */

 #ifndef DRAWER_RVIZ_H
 #define DRAWER_RVIZ_H
 
 #include "tracking/drawer.h"
 
 #include <atomic>
 #include <condition_variable>
 #include <memory>
 #include <mutex>
 
 #include <nav_msgs/Path.h>
 #include <ros/ros.h>
 #include <sensor_msgs/Image.h>  // ★★★ 新增 ★★★
 #include <cv_bridge/cv_bridge.h> // ★★★ 新增 ★★★
 
 using std::string;
 using std::vector;
 
 class DrawerRviz : public Drawer {
 
 public:
     explicit DrawerRviz(ros::NodeHandle &nh);
 
     void run() override;
 
     void setFinished() override;
 
     // 地圖
     void addNewFixedMappoint(Vector3d point) override;
 
     void updateMap(const Eigen::Matrix4d &pose) override;
 
     // GPS路径 (输入已经是ENU坐标)
     void updateGnssPosition(const Eigen::Vector3d &enu) override;
 
     // 跟蹤圖像
     void updateFrame(Frame::Ptr frame) override;
     void updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                 vector<MapPointType> mappoint_type) override;
     void updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) override;
 
 private:
     void publishTrackingImage();
 
     void publishMapPoints();
 
     void publishOdometry();
     
     // ★★★ 新增：YOLO 遮罩回調函數 ★★★
     void yoloMaskCallback(const sensor_msgs::ImageConstPtr& mask_msg);
 
 private:
     // 多線程
     std::condition_variable update_sem_;
     std::mutex update_mutex_;
     std::mutex map_mutex_;
     std::mutex image_mutex_;
     std::mutex mask_mutex_;  // ★★★ 新增：保護 YOLO 遮罩 ★★★
 
     // 標誌
     std::atomic<bool> isfinished_;
     std::atomic<bool> isframerdy_;
     std::atomic<bool> ismaprdy_;
 
     // 跟蹤
     Mat raw_image_;
     cv::Mat yolo_mask_;      // ★★★ 新增：YOLO 遮罩 ★★★
     vector<Vector3d> fixed_mappoints_;
 
     Pose pose_;
     nav_msgs::Path path_;
     nav_msgs::Path gnss_path_;  // GPS路径
 
     ros::Publisher path_pub_;
     ros::Publisher pose_pub_;
     ros::Publisher track_image_pub_;
     ros::Publisher fixed_points_pub_;
     ros::Publisher current_points_pub_;
     ros::Publisher gnss_path_pub_;  // GPS路径发布器
 
     ros::Subscriber yolo_mask_sub_;  // ★★★ 新增：訂閱 YOLO 遮罩 ★★★
 
     string frame_id_;
 };
 
 #endif // DRAWER_RVIZ_H