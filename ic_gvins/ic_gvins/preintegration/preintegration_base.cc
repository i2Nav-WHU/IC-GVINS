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

#include "preintegration/preintegration_base.h"

PreintegrationBase::PreintegrationBase(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0,
                                       IntegrationState state)
    : parameters_(std::move(parameters))
    , current_state_(std::move(state)) {

    start_time_ = imu0.time;
    end_time_   = imu0.time;

    imu_buffer_.clear();
    imu_buffer_.push_back(imu0);

    gravity_ = Vector3d(0, 0, parameters_->gravity);
}

void PreintegrationBase::integration(const IMU &imu_pre, const IMU &imu_cur) {
    // 区间时间累积
    double dt = imu_cur.dt;
    delta_time_ += dt;

    end_time_           = imu_cur.time;
    current_state_.time = imu_cur.time;

    // 连续状态积分, 先位置速度再姿态

    // 位置速度
    Vector3d dvfb = imu_cur.dvel + 0.5 * imu_cur.dtheta.cross(imu_cur.dvel) +
                    1.0 / 12.0 * (imu_pre.dtheta.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dtheta));
    Vector3d dvel = current_state_.q.toRotationMatrix() * dvfb + gravity_ * dt;

    current_state_.p += dt * current_state_.v + 0.5 * dt * dvel;
    current_state_.v += dvel;

    // 姿态
    Vector3d dtheta = imu_cur.dtheta + 1.0 / 12.0 * imu_pre.dtheta.cross(imu_cur.dtheta);
    current_state_.q *= Rotation::rotvec2quaternion(dtheta);
    current_state_.q.normalize();

    // 预积分
    dvel = delta_state_.q.toRotationMatrix() * dvfb;
    delta_state_.p += dt * delta_state_.v + 0.5 * dt * dvel;
    delta_state_.v += dvel;

    // 姿态
    delta_state_.q *= Rotation::rotvec2quaternion(dtheta);
    delta_state_.q.normalize();
}

void PreintegrationBase::addNewImu(const IMU &imu) {
    imu_buffer_.push_back(imu);
    integrationProcess(imu_buffer_.size() - 1);
}

void PreintegrationBase::reintegration(IntegrationState &state) {
    current_state_ = std::move(state);
    resetState(current_state_);

    for (size_t k = 1; k < imu_buffer_.size(); k++) {
        integrationProcess(k);
    }
}

IMU PreintegrationBase::compensationBias(const IMU &imu) const {
    IMU imu_calib = imu;
    imu_calib.dtheta -= imu_calib.dt * delta_state_.bg;
    imu_calib.dvel -= imu_calib.dt * delta_state_.ba;

    return imu_calib;
}

IMU PreintegrationBase::compensationScale(const IMU &imu) const {
    IMU imu_calib = imu;

    for (int k = 0; k < 3; k++) {
        imu_calib.dtheta[k] *= (1.0 - delta_state_.sg[k]);
        imu_calib.dvel[k] *= (1.0 - delta_state_.sa[k]);
    }
    return imu_calib;
}

void PreintegrationBase::stateToData(const IntegrationState &state, IntegrationStateData &data) {
    data.time = state.time;

    memcpy(data.pose, state.p.data(), sizeof(double) * 3);
    memcpy(data.pose + 3, state.q.coeffs().data(), sizeof(double) * 4);

    memcpy(data.mix, state.v.data(), sizeof(double) * 3);
    memcpy(data.mix + 3, state.bg.data(), sizeof(double) * 3);
    memcpy(data.mix + 6, state.ba.data(), sizeof(double) * 3);
}

void PreintegrationBase::stateFromData(const IntegrationStateData &data, IntegrationState &state) {
    state.time = data.time;

    memcpy(state.p.data(), data.pose, sizeof(double) * 3);
    memcpy(state.q.coeffs().data(), data.pose + 3, sizeof(double) * 4);
    state.q.normalize();

    memcpy(state.v.data(), data.mix, sizeof(double) * 3);
    memcpy(state.bg.data(), data.mix + 3, sizeof(double) * 3);
    memcpy(state.ba.data(), data.mix + 6, sizeof(double) * 3);
}