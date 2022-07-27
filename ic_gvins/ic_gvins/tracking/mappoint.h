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

#ifndef GVINS_MAPPOINT_H
#define GVINS_MAPPOINT_H

#include "tracking/camera.h"
#include "tracking/feature.h"

#include <Eigen/Geometry>
#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

using cv::Mat;
using Eigen::Vector3d;

enum MapPointType {
    MAPPOINT_NONE              = -1,
    MAPPOINT_TRIANGULATED      = 0,
    MAPPOINT_DEPTH_ASSOCIATED  = 1,
    MAPPOINT_DEPTH_INITIALIZED = 2,
    MAPPOINT_FIXED             = 3,
};

class MapPoint {

public:
    typedef std::shared_ptr<MapPoint> Ptr;

    static constexpr double DEFAULT_DEPTH  = 10.0;
    static constexpr double NEAREST_DEPTH  = 1;   // 最近可用路标点深度
    static constexpr double FARTHEST_DEPTH = 200; // 最远可用路标点深度

    MapPoint() = delete;
    MapPoint(ulong id, const std::shared_ptr<Frame> &ref_frame, Vector3d pos, cv::Point2f keypoint, double depth,
             MapPointType type);

    Vector3d &pos() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return pos_;
    };

    int observedTimes() const {
        return observed_times_;
    }

    ulong id() const {
        return id_;
    }

    static MapPoint::Ptr createMapPoint(std::shared_ptr<Frame> &ref_frame, Vector3d &pos, cv::Point2f &keypoint,
                                        double depth, MapPointType type);

    void addObservation(const Feature::Ptr &feature);

    void increaseUsedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        used_times_++;
    }

    void decreaseUsedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        if (used_times_) {
            used_times_--;
        }
    }

    int usedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return used_times_;
    }

    void addOptimizedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        optimized_times_++;
    }

    int optimizedTimes() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return optimized_times_;
    }

    void removeAllObservations() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        observations_.clear();
    }

    std::vector<std::weak_ptr<Feature>> observations() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return observations_;
    }

    void setOutlier(bool isoutlier) {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);

        isoutlier_ = isoutlier;
    }

    bool isOutlier() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return isoutlier_;
    }

    void setReferenceFrame(const std::shared_ptr<Frame> &frame, Vector3d pos, cv::Point2f keypoint, double depth,
                           MapPointType type);

    double depth() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return depth_;
    }

    void updateDepth(double depth) {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        depth_ = depth;
    }

    ulong referenceFrameId();

    MapPointType &mapPointType() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return mappoint_type_;
    }

    std::shared_ptr<Frame> referenceFrame() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return ref_frame_.lock();
    }

    const cv::Point2f &referenceKeypoint() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return ref_frame_keypoint_;
    }

    bool isNeedUpdate() {
        std::unique_lock<std::mutex> lock(mappoint_mutex_);
        return isneedupdate_;
    }

private:
    std::vector<std::weak_ptr<Feature>> observations_;

    std::mutex mappoint_mutex_;
    bool isneedupdate_{false};

    Vector3d pos_, pos_tmp_;

    // 参考帧中的深度
    double depth_{DEFAULT_DEPTH}, depth_tmp_{DEFAULT_DEPTH};
    cv::Point2f ref_frame_keypoint_, ref_frame_keypoint_tmp_;
    std::weak_ptr<Frame> ref_frame_, ref_frame_tmp_;

    int optimized_times_;
    int used_times_;
    int observed_times_;
    bool isoutlier_;

    ulong id_;
    MapPointType mappoint_type_{MAPPOINT_NONE}, mappoint_type_tmp_{MAPPOINT_NONE};
};

#endif // GVINS_MAPPOINT_H
