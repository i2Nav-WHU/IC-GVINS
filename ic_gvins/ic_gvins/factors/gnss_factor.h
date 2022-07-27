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

#ifndef GNSS_FACTOR_H
#define GNSS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "common/types.h"

class GnssFactor : public ceres::SizedCostFunction<3, 7> {

public:
    explicit GnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    void updateGnssState(const GNSS &gnss) {
        gnss_ = gnss;
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);

        error = p + q.toRotationMatrix() * lever_ - gnss_.blh;

        Matrix3d sqrt_info_ = Matrix3d::Zero();
        sqrt_info_(0, 0)    = 1.0 / gnss_.std[0];
        sqrt_info_(1, 1)    = 1.0 / gnss_.std[1];
        sqrt_info_(2, 2)    = 1.0 / gnss_.std[2];

        error = sqrt_info_ * error;

        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);
                jacobian_pose.setZero();

                jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
                jacobian_pose.block<3, 3>(0, 3) = -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);

                jacobian_pose = sqrt_info_ * jacobian_pose;
            }
        }

        return true;
    }

private:
    GNSS gnss_;
    Vector3d lever_;
};

#endif // GNSS_FACTOR_H
