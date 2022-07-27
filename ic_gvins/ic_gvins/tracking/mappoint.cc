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

#include "tracking/mappoint.h"
#include "tracking/frame.h"

MapPoint::MapPoint(ulong id, const std::shared_ptr<Frame> &ref_frame, Vector3d pos, cv::Point2f keypoint, double depth,
                   MapPointType type)
    : pos_(std::move(pos))
    , depth_(depth)
    , ref_frame_keypoint_(std::move(keypoint))
    , ref_frame_(ref_frame)
    , optimized_times_(0)
    , used_times_(0)
    , observed_times_(0)
    , isoutlier_(false)
    , id_(id)
    , mappoint_type_(type) {

    // 防止深度错误
    if ((depth_ < NEAREST_DEPTH) || (depth_ > FARTHEST_DEPTH)) {
        depth_ = DEFAULT_DEPTH;
    }
}

MapPoint::Ptr MapPoint::createMapPoint(std::shared_ptr<Frame> &ref_frame, Vector3d &pos, cv::Point2f &feature,
                                       double depth, MapPointType type) {
    static ulong factory_id_ = 0;
    return std::make_shared<MapPoint>(factory_id_++, ref_frame, pos, feature, depth, type);
}

void MapPoint::addObservation(const Feature::Ptr &feature) {
    std::unique_lock<std::mutex> lock(mappoint_mutex_);

    observations_.push_back(feature);
    observed_times_++;
}

void MapPoint::setReferenceFrame(const std::shared_ptr<Frame> &frame, Vector3d pos, cv::Point2f keypoint, double depth,
                                 MapPointType type) {
    std::unique_lock<std::mutex> lock(mappoint_mutex_);

    depth_tmp_ = depth;
    if (depth_tmp_ < 1.0) {
        depth_tmp_ = DEFAULT_DEPTH;
    }

    pos_tmp_                = std::move(pos);
    ref_frame_tmp_          = frame;
    ref_frame_keypoint_tmp_ = std::move(keypoint);
    mappoint_type_tmp_      = type;
    isneedupdate_           = true;
}

ulong MapPoint::referenceFrameId() {
    std::unique_lock<std::mutex> lock(mappoint_mutex_);
    auto frame = ref_frame_.lock();
    if (frame) {
        return frame->id();
    }

    return 0;
}
