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

#ifndef GVINS_CAMERA_H
#define GVINS_CAMERA_H

#include "common/types.h"

#include <memory>
#include <opencv2/opencv.hpp>

using cv::Mat;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;

class Camera {

public:
    typedef std::shared_ptr<Camera> Ptr;

    Camera() = delete;

    Camera(Mat intrinsic, Mat distortion, const cv::Size &size);

    static Camera::Ptr createCamera(const std::vector<double> &intrinsic, const std::vector<double> &distortion,
                                    const std::vector<int> &size);

    const Mat &cameraMatrix() {
        return intrinsic_;
    }

    void undistortPoints(std::vector<cv::Point2f> &pts);
    void undistortImage(const Mat &src, Mat &dst);
    void distortPoints(std::vector<cv::Point2f> &pts) const;
    void distortPoint(cv::Point2f &pp) const;
    cv::Point2f distortCameraPoint(const Vector3d &pc) const;

    Vector2d reprojectionError(const Pose &pose, const Vector3d &pw, const cv::Point2f &pp) const;

    static Vector3d world2cam(const Vector3d &world, const Pose &pose);
    static Vector3d cam2world(const Vector3d &cam, const Pose &pose);

    Vector3d pixel2cam(const cv::Point2f &pixel) const;
    Vector3d pixel2unitcam(const cv::Point2f &pixel) const;
    cv::Point2f cam2pixel(const Vector3d &cam) const;

    Vector3d pixel2world(const cv::Point2f &pixel, const Pose &pose) const;
    cv::Point2f world2pixel(const Vector3d &world, const Pose &pose) const;

    cv::Size size() const {
        return {width_, height_};
    }

    int width() const {
        return width_;
    }

    int height() const {
        return height_;
    }

    double focalLength() const {
        return (fx_ + fy_) * 0.5;
    }

private:
    Mat distortion_;
    Mat undissrc_, undisdst_;

    double fx_, fy_, cx_, cy_, skew_;
    double k1_, k2_, k3_, p1_, p2_;
    Mat intrinsic_;

    int width_, height_;
};

#endif // GVINS_CAMERA_H
