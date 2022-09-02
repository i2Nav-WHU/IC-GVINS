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

#ifndef IMU_ERROR_FACTOR_H
#define IMU_ERROR_FACTOR_H

#include "preintegration/preintegration_base.h"

#include <ceres/ceres.h>

class ImuErrorFactor : public ceres::CostFunction {

public:
    explicit ImuErrorFactor(Preintegration::PreintegrationOptions options)
        : options_(options) {

        *mutable_parameter_block_sizes() = vector<int>{Preintegration::numMixParameter(options_)};

        if ((options_ == Preintegration::PREINTEGRATION_NORMAL) || (options_ == Preintegration::PREINTEGRATION_EARTH)) {
            set_num_residuals(6);
        } else {
            set_num_residuals(7);
        }
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {

        // parameters: vel[3], bg[3], ba[3], sodo

        // bg, ba
        for (size_t k = 0; k < 3; k++) {
            residuals[k + 0] = parameters[0][k + 3] / IMU_GRY_BIAS_STD;
            residuals[k + 3] = parameters[0][k + 6] / IMU_ACC_BIAS_STD;
        }

        if ((options_ == Preintegration::PREINTEGRATION_NORMAL) || (options_ == Preintegration::PREINTEGRATION_EARTH)) {
            // Without odometer

            if (jacobians && jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 6, 9, Eigen::RowMajor>> jaco(jacobians[0]);
                jaco.setZero();

                for (size_t k = 0; k < 3; k++) {
                    jaco(k + 0, k + 3) = 1.0 / IMU_GRY_BIAS_STD;
                    jaco(k + 3, k + 6) = 1.0 / IMU_ACC_BIAS_STD;
                }
            }

        } else if ((options_ == Preintegration::PREINTEGRATION_ODO) ||
                   (options_ == Preintegration::PREINTEGRATION_EARTH_ODO)) {
            // With odometer
            residuals[6] = parameters[0][9] / ODO_SCALE_STD;

            if (jacobians && jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 7, 10, Eigen::RowMajor>> jaco(jacobians[0]);
                jaco.setZero();

                for (size_t k = 0; k < 3; k++) {
                    jaco(k + 0, k + 3) = 1.0 / IMU_GRY_BIAS_STD;
                    jaco(k + 3, k + 6) = 1.0 / IMU_ACC_BIAS_STD;
                }
                jaco(6, 9) = 1.0 / ODO_SCALE_STD;
            }
        }

        return true;
    }

private:
    static constexpr double IMU_GRY_BIAS_STD = 7200 / 3600.0 * M_PI / 180.0; // 7200 deg / hr
    static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5;               // 20000 mGal
    static constexpr double ODO_SCALE_STD    = 2.0e4 * 1.0e-6;               // 0.02

    Preintegration::PreintegrationOptions options_;
};

#endif // IMU_ERROR_FACTOR_H
