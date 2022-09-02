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

#ifndef PREINTEGRATION_NORMAL_H
#define PREINTEGRATION_NORMAL_H

#include "preintegration/preintegration_base.h"

class PreintegrationNormal : public PreintegrationBase {

public:
    PreintegrationNormal(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0, IntegrationState state);

    Eigen::MatrixXd evaluate(const IntegrationState &state0, const IntegrationState &state1,
                             double *residuals) override;

    Eigen::MatrixXd residualJacobianPose0(const IntegrationState &state0, const IntegrationState &state1,
                                          double *jacobian) override;
    Eigen::MatrixXd residualJacobianPose1(const IntegrationState &state0, const IntegrationState &state1,
                                          double *jacobian) override;
    Eigen::MatrixXd residualJacobianMix0(const IntegrationState &state0, const IntegrationState &state1,
                                         double *jacobian) override;
    Eigen::MatrixXd residualJacobianMix1(const IntegrationState &state0, const IntegrationState &state1,
                                         double *jacobian) override;
    int numResiduals() override;
    int numMixParametersBlocks() override;
    vector<int> numBlocksParameters() override;

    static IntegrationStateData stateToData(const IntegrationState &state);
    static IntegrationState stateFromData(const IntegrationStateData &data);
    void constructState(const double *const *parameters, IntegrationState &state0, IntegrationState &state1) override;

protected:
    void integrationProcess(unsigned long index) override;
    void resetState(const IntegrationState &state) override;

    void updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) override;

private:
    void resetState(const IntegrationState &state, int num);
    void setNoiseMatrix();

public:
    static constexpr int NUM_MIX = 9;

private:
    static constexpr int NUM_STATE = 15;
    static constexpr int NUM_NOISE = 12;
};

#endif // PREINTEGRATION_NORMAL_H
