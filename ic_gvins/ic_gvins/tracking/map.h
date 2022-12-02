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

#ifndef GVINS_MAP_H
#define GVINS_MAP_H

#include "tracking/frame.h"
#include "tracking/mappoint.h"

#include <memory>
#include <mutex>
#include <unordered_map>

class Map {

public:
    typedef std::shared_ptr<Map> Ptr;

    typedef std::unordered_map<ulong, Frame::Ptr> KeyFrames;
    typedef std::unordered_map<ulong, MapPoint::Ptr> LandMarks;

    Map() = delete;
    explicit Map(size_t size)
        : window_size_(size) {
    }

    void resetWindowSize(size_t size) {
        window_size_ = size;
    }

    size_t windowSize() const {
        return window_size_;
    }

    void insertKeyFrame(const Frame::Ptr &frame);

    const KeyFrames &keyframes() {
        return keyframes_;
    }

    const LandMarks &landmarks() {
        return landmarks_;
    }

    vector<ulong> orderedKeyFrames();
    Frame::Ptr oldestKeyFrame();
    const Frame::Ptr &latestKeyFrame();

    void removeMappoint(MapPoint::Ptr &mappoint);
    void removeKeyFrame(Frame::Ptr &frame, bool isremovemappoint);

    double mappointObservedRate(const MapPoint::Ptr &mappoint);

    bool isMaximumKeframes() {
        std::unique_lock<std::mutex> lock(map_mutex_);
        return keyframes_.size() > window_size_;
    }

    bool isKeyFrameInMap(const Frame::Ptr &frame) {
        std::unique_lock<std::mutex> lock(map_mutex_);
        return keyframes_.find(frame->keyFrameId()) != keyframes_.end();
    }

    bool isWindowFull() {
        std::unique_lock<std::mutex> lock(map_mutex_);
        return is_window_full_;
    }

    bool isWindowNormal() {
        std::unique_lock<std::mutex> lock(map_mutex_);
        return keyframes_.size() == window_size_;
    }

private:
    std::mutex map_mutex_;

    KeyFrames keyframes_;
    LandMarks landmarks_;

    Frame::Ptr latest_keyframe_;

    size_t window_size_{20};
    bool is_window_full_{false};
};

#endif // GVINS_MAP_H
