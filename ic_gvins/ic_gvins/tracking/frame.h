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

#ifndef GVINS_FRAME_H
#define GVINS_FRAME_H

#include "common/types.h"
#include "tracking/feature.h"

#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

using cv::Mat;
using std::vector;

enum keyFrameState {
    KEYFRAME_NONE              = 0,
    KEYFRAME_REMOVE_SECOND_NEW = 1,
    KEYFRAME_NORMAL            = 2,
    KEYFRAME_REMOVE_OLDEST     = 3,
};

class Frame {

public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame() = delete;
    Frame(ulong id, double stamp, Mat image);

    static Frame::Ptr createFrame(double stamp, const Mat &image);

    void setKeyFrame(int state);

    void resetKeyFrame() {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        iskeyframe_     = false;
        keyframe_state_ = KEYFRAME_NONE;
    }

    Mat &image() {
        return image_;
    }

    Mat &rawImage() {
        return raw_image_;
    }

    Pose pose() {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        return pose_;
    }

    void setPose(Pose pose) {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        pose_ = std::move(pose);
    }

    std::unordered_map<ulong, Feature::Ptr> features() {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        return features_;
    }

    void clearFeatures() {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        features_.clear();
        unupdated_mappoints_.clear();
    }

    size_t numFeatures() {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        return features_.size();
    }

    const std::vector<std::shared_ptr<MapPoint>> &unupdatedMappoints() {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        return unupdated_mappoints_;
    }

    void addNewUnupdatedMappoint(const std::shared_ptr<MapPoint> &mappoint) {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        unupdated_mappoints_.push_back(mappoint);
    }

    void addFeature(ulong mappointid, const Feature::Ptr &features) {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        features_.insert(make_pair(mappointid, features));
    }

    double stamp() const {
        return stamp_;
    }

    void setStamp(double stamp) {
        stamp_ = stamp;
    }

    double timeDelay() const {
        return td_;
    }

    void setTimeDelay(double td) {
        td_ = td;
    }

    bool isKeyFrame() const {
        return iskeyframe_;
    }

    ulong id() const {
        return id_;
    }

    ulong keyFrameId() const {
        return keyframe_id_;
    }

    void setKeyFrameState(int state) {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        keyframe_state_ = state;
    }

    int keyFrameState() {
        std::unique_lock<std::mutex> lock(frame_mutex_);

        return keyframe_state_;
    }

private:
    int keyframe_state_{KEYFRAME_NORMAL};

    std::mutex frame_mutex_;

    ulong id_;
    ulong keyframe_id_;

    double stamp_;
    double td_{0};

    Pose pose_;

    Mat image_, raw_image_;

    bool iskeyframe_;

    std::unordered_map<ulong, Feature::Ptr> features_;
    vector<std::shared_ptr<MapPoint>> unupdated_mappoints_;
};

#endif // GVINS_FRAME_H
