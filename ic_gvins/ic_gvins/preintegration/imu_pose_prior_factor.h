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

#include "common/rotation.h"

#include <ceres/ceres.h>

class ImuPosePriorFactor : public ceres::CostFunction {

public:
    ImuPosePriorFactor(double *pose, double *std) {
        memcpy(pose_, pose, sizeof(double) * 7);

        sqrt_info_.setZero();
        for (size_t k = 0; k < 6; k++) {
            sqrt_info_(k, k) = 1.0 / std[k];
        }

        *mutable_parameter_block_sizes() = vector<int>{7};
        set_num_residuals(6);
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

        // Position
        for (size_t k = 0; k < 3; k++) {
            residual(k, 0) = (parameters[0][k] - pose_[k]);
        }

        // Attitude
        Quaterniond q_p(pose_[6], pose_[3], pose_[4], pose_[5]);
        Quaterniond q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        residual.block<3, 1>(3, 0) = 2 * (q.inverse() * q_p).vec();

        residual = sqrt_info_ * residual;

        if (jacobians && jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jaco(jacobians[0]);
            jaco.setZero();

            jaco.block<3, 3>(0, 0) = Matrix3d::Identity();
            jaco.block<3, 3>(3, 3) = -Rotation::quaternionright(q.inverse() * q_p).bottomRightCorner<3, 3>();

            jaco = sqrt_info_ * jaco;
        }

        return true;
    }

private:
    double pose_[7];

    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> sqrt_info_;
};