#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from ultralytics import YOLO

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class YOLOMaskNode:
    def __init__(self):
        rospy.init_node('yolo_mask_node', anonymous=True)
        rospy.loginfo("YOLO Mask Node starting...")

        # --- 參數 ---
        model_path = rospy.get_param('~model_path', 'yolo11n-seg_16.engine')
        image_topic = rospy.get_param('~image_topic', '/cam0')
        self.mask_topic = rospy.get_param('~mask_topic', '/yolo/mask')
        self.conf_threshold = rospy.get_param('~confidence_threshold', 0.3)
        
        # 移除與複雜運動判斷相關的參數
        # self.relative_motion_threshold = rospy.get_param('~relative_motion_threshold', 10.0)
        # self.stationary_threshold = rospy.get_param('~stationary_threshold', 1.0)
        
        # 這是我們需要遮蔽的目標類別 (例如：人、車、自行車、卡車等)
        self.target_classes = {0, 1, 2, 3, 5, 7}

        # --- YOLO 模型 ---
        try:
            self.model = YOLO(model_path)
            rospy.loginfo(f"YOLO model loaded: {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            return

        # --- ROS ---
        self.bridge = CvBridge()
        self.mask_pub = rospy.Publisher(self.mask_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            image_topic, Image, self.image_callback, 
            queue_size=1, buff_size=2**24
        )
        
        # 移除光流與追蹤歷史相關的變數
        self.prev_gray = None
        # self.track_history = defaultdict(lambda: [])
        # self.stationary_time = defaultdict(float)
        # self.last_update_time = {}

    def image_callback(self, ros_image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")
            
            # 移除背景運動估計
            # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # camera_motion = self.estimate_camera_motion(gray)
            
            # --- YOLO tracking (現在只是普通的檢測與分割) ---
            results = self.model.track(
                source=cv_image,
                conf=self.conf_threshold,
                classes=list(self.target_classes),
                device='cuda:0',
                verbose=False,
                persist=True # 保持追蹤 ID，雖然不再用於判斷動靜態，但有助於 ID 穩定
            )

            # --- 產生簡化遮罩 ---
            # 任何被檢測到的目標類別都視為動態物體
            final_mask = self.create_simple_mask(results, cv_image.shape)
            
            # 發布
            mask_msg = self.bridge.cv2_to_imgmsg(final_mask, "mono8")
            mask_msg.header.stamp = ros_image_msg.header.stamp
            mask_msg.header.frame_id = ros_image_msg.header.frame_id
            self.mask_pub.publish(mask_msg)
            
            # 移除上一幀的更新
            # self.prev_gray = gray
            
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    # 移除 estimate_camera_motion 函式

    def create_simple_mask(self, results, image_shape):
        """
        根據 YOLO 分割結果，將所有目標類別的區域遮蔽 (設為 0)
        """
        combined_mask = np.full((image_shape[0], image_shape[1]), 255, dtype=np.uint8)

        if not results or results[0].masks is None:
            return combined_mask

        # 遍歷所有分割結果 (無論是否有追蹤 ID)
        for seg in results[0].masks.xy:
            polygon = seg.astype(np.int32).reshape((-1, 1, 2))
            # 簡單地將多邊形內部設為 0 (動態/需要遮蔽)
            cv2.fillPoly(combined_mask, [polygon], 0)

        # 膨脹處理，確保遮罩邊緣能夠覆蓋所有特徵點
        kernel = np.ones((7, 7), np.uint8)
        # combined_mask: 0=dynamic, 255=static. Invert for dilation.
        inverted_mask = cv2.bitwise_not(combined_mask)
        dilated_mask_inv = cv2.dilate(inverted_mask, kernel, iterations=1)
        final_mask = cv2.bitwise_not(dilated_mask_inv)
        
        return final_mask

    # 移除 update_motion_state 函式


def main():
    try:
        node = YOLOMaskNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLO Mask Node shutting down.")

if __name__ == '__main__':
    main()