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

#ifndef FUSION_ROS_H
#define FUSION_ROS_H

#include "ic_gvins/common/types.h"
#include "ic_gvins/ic_gvins.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>

class FusionROS {

public:
    FusionROS() = default;

    void run();

    void setFinished();

private:
    void imuCallback(const sensor_msgs::ImuConstPtr &imumsg);

    void gnssCallback(const sensor_msgs::NavSatFixConstPtr &gnssmsg);

    void imageCallback(const sensor_msgs::ImageConstPtr &imagemsg);

private:
    std::shared_ptr<GVINS> gvins_;

    IMU imu_{.time = 0}, imu_pre_{.time = 0};
    Frame::Ptr frame_;
    GNSS gnss_;

    bool isusegnssoutage_{false};
    double gnssoutagetime_{0};
    double gnssthreshold_{20.0};

    std::queue<IMU> imu_buffer_;
    std::queue<Frame::Ptr> frame_buffer_;
};

#endif // FUSION_ROS_H
