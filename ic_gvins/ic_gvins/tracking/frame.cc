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

#include "tracking/frame.h"

Frame::Frame(ulong id, double stamp, Mat image)
    : id_(id)
    , keyframe_id_(0)
    , stamp_(stamp)
    , image_(std::move(image))
    , iskeyframe_(false) {
    features_.clear();
    unupdated_mappoints_.clear();

    image_.copyTo(raw_image_);
}

Frame::Ptr Frame::createFrame(double stamp, const Mat &image) {
    static ulong factory_id = 0;

    return std::make_shared<Frame>(factory_id++, stamp, image);
}

void Frame::setKeyFrame(int state) {
    std::unique_lock<std::mutex> lock(frame_mutex_);

    static ulong keyframe_factory_id = 0;

    if (!iskeyframe_) {
        iskeyframe_     = true;
        keyframe_id_    = keyframe_factory_id++;
        keyframe_state_ = state;
    }
}
