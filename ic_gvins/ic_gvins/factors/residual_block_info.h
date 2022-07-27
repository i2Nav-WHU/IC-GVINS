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

#ifndef RESIDUAL_BLOCK_INFO_H
#define RESIDUAL_BLOCK_INFO_H

#define POSE_LOCAL_SIZE 6
#define POSE_GLOBAL_SIZE 7

#include <ceres/ceres.h>
#include <memory>

class ResidualBlockInfo {

public:
    ResidualBlockInfo(std::shared_ptr<ceres::CostFunction> cost_function,
                      std::shared_ptr<ceres::LossFunction> loss_function, std::vector<double *> parameter_blocks,
                      std::vector<int> marg_para_index)
        : cost_function_(std::move(cost_function))
        , loss_function_(std::move(loss_function))
        , parameter_blocks_(std::move(parameter_blocks))
        , marg_para_index_(std::move(marg_para_index)) {
    }

    void Evaluate() {
        residuals_.resize(cost_function_->num_residuals());

        std::vector<int> block_sizes = cost_function_->parameter_block_sizes();
        auto raw_jacobians           = new double *[block_sizes.size()];
        jacobians_.resize(block_sizes.size());

        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
            jacobians_[i].resize(cost_function_->num_residuals(), block_sizes[i]);
            raw_jacobians[i] = jacobians_[i].data();
        }
        cost_function_->Evaluate(parameter_blocks_.data(), residuals_.data(), raw_jacobians);

        delete[] raw_jacobians;

        if (loss_function_) {
            // 鲁棒核函数调整, 参考ceres/internal/ceres/corrector.cc
            double residual_scaling, alpha_sq_norm;

            double sq_norm, rho[3];

            sq_norm = residuals_.squaredNorm();
            loss_function_->Evaluate(sq_norm, rho);

            double sqrt_rho1 = sqrt(rho[1]);

            if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
                residual_scaling = sqrt_rho1;
                alpha_sq_norm    = 0.0;
            } else {
                // 0.5 *  alpha^2 - alpha - rho'' / rho' *  z'z = 0
                const double D     = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
                const double alpha = 1.0 - sqrt(D);
                residual_scaling   = sqrt_rho1 / (1 - alpha);
                alpha_sq_norm      = alpha / sq_norm;
            }

            for (size_t i = 0; i < parameter_blocks_.size(); i++) {
                // J = sqrt_rho1 * (J - alpha_sq_norm * r* (r.transpose() * J))
                jacobians_[i] =
                    sqrt_rho1 * (jacobians_[i] - alpha_sq_norm * residuals_ * (residuals_.transpose() * jacobians_[i]));
            }
            residuals_ *= residual_scaling;
        }
    }

    const std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> &jacobians() {
        return jacobians_;
    }

    const std::vector<int> &parameterBlockSizes() {
        return cost_function_->parameter_block_sizes();
    }

    const std::vector<double *> &parameterBlocks() {
        return parameter_blocks_;
    }

    const Eigen::VectorXd &residuals() {
        return residuals_;
    }

    const std::vector<int> &marginalizationParametersIndex() {
        return marg_para_index_;
    }

private:
    std::shared_ptr<ceres::CostFunction> cost_function_;
    std::shared_ptr<ceres::LossFunction> loss_function_;

    std::vector<double *> parameter_blocks_;

    std::vector<int> marg_para_index_;

    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians_;
    Eigen::VectorXd residuals_;
};

#endif // RESIDUAL_BLOCK_INFO_H
