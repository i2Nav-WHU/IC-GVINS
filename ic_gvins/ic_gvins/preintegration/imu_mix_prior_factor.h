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

#include "preintegration/preintegration.h"

#include <ceres/ceres.h>

class ImuMixPriorFactor : public ceres::CostFunction {

public:
    ImuMixPriorFactor(Preintegration::PreintegrationOptions options, const double *mix, const double *mix_std)
        : options_(options) {

        memcpy(mix_, mix, sizeof(double) * 18);
        memcpy(mix_std_, mix_std, sizeof(double) * 18);

        *mutable_parameter_block_sizes() = vector<int>{Preintegration::numMixParameter(options_)};
        set_num_residuals(Preintegration::numMixParameter(options_));
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {

        // parameters: vel[3], bg[3], ba[3], sodo, abv

        if (options_ == Preintegration::PREINTEGRATION_NORMAL || options_ == Preintegration::PREINTEGRATION_EARTH) {
            // vel, bg, ba
            for (size_t k = 0; k < 9; k++) {
                residuals[k] = (parameters[0][k] - mix_[k]) / mix_std_[k];
            }

            if (jacobians && jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>> jaco(jacobians[0]);
                jaco.setZero();
                for (size_t k = 0; k < 9; k++) {
                    jaco(k, k) = 1.0 / mix_std_[k];
                }
            }
        } else if (options_ == Preintegration::PREINTEGRATION_ODO ||
                   options_ == Preintegration::PREINTEGRATION_EARTH_ODO) {
            // vel, bg, ba, sodo
            for (size_t k = 0; k < 10; k++) {
                residuals[k] = (parameters[0][k] - mix_[k]) / mix_std_[k];
            }

            if (jacobians && jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 10, 10, Eigen::RowMajor>> jaco(jacobians[0]);
                jaco.setZero();

                for (size_t k = 0; k < 10; k++) {
                    jaco(k, k) = 1.0 / mix_std_[k];
                }
            }
        }

        return true;
    }

private:
    Preintegration::PreintegrationOptions options_;

    double mix_[18], mix_std_[18];
};
