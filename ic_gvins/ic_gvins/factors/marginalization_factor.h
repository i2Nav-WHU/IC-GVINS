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

#ifndef MARGINALIZATION_FACTOR_H
#define MARGINALIZATION_FACTOR_H

#include "factors/marginalization_info.h"

#include <ceres/ceres.h>
#include <memory>

class MarginalizationFactor : public ceres::CostFunction {

public:
    MarginalizationFactor() = delete;
    explicit MarginalizationFactor(std::shared_ptr<MarginalizationInfo> marg_info)
        : marg_info_(std::move(marg_info)) {

        // 给定每个参数块数据大小
        for (auto size : marg_info_->remainedBlockSize()) {
            mutable_parameter_block_sizes()->push_back(size);
        }

        // 残差大小
        set_num_residuals(marg_info_->remainedSize());
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        int marginalizaed_size = marg_info_->marginalizedSize();
        int remained_size      = marg_info_->remainedSize();

        const vector<int> &remained_block_index     = marg_info_->remainedBlockIndex();
        const vector<int> &remained_block_size      = marg_info_->remainedBlockSize();
        const vector<double *> &remained_block_data = marg_info_->remainedBlockData();

        Eigen::VectorXd dx(remained_size);
        for (size_t i = 0; i < remained_block_size.size(); i++) {
            int size  = remained_block_size[i];
            int index = remained_block_index[i] - marginalizaed_size;

            Eigen::VectorXd x  = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
            Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(remained_block_data[i], size);

            // dx = x - x0
            if (size == POSE_GLOBAL_SIZE) {
                Eigen::Quaterniond dq(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                                      Eigen::Quaterniond(x(6), x(3), x(4), x(5)));

                dx.segment(index, 3)     = x.head<3>() - x0.head<3>();
                dx.segment(index + 3, 3) = 2.0 * dq.vec();
                if (dq.w() < 0) {
                    dx.segment<3>(index + 3) = -2.0 * dq.vec();
                }
            } else {
                dx.segment(index, size) = x - x0;
            }
        }

        // e = e0 + J0 * dx
        Eigen::Map<Eigen::VectorXd>(residuals, remained_size) =
            marg_info_->linearizedResiduals() + marg_info_->linearizedJacobians() * dx;

        if (jacobians) {

            for (size_t i = 0; i < remained_block_size.size(); i++) {
                if (jacobians[i]) {
                    int size       = remained_block_size[i];
                    int index      = remained_block_index[i] - marginalizaed_size;
                    int local_size = marg_info_->localSize(size);

                    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(
                        jacobians[i], remained_size, size);

                    // J = J0
                    jacobian.setZero();
                    jacobian.leftCols(local_size) = marg_info_->linearizedJacobians().middleCols(index, local_size);
                }
            }
        }

        return true;
    }

private:
    std::shared_ptr<MarginalizationInfo> marg_info_;
};

#endif // MARGINALIZATION_FACTOR_H
