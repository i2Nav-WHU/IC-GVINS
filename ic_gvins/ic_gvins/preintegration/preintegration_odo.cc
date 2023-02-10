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

#include "preintegration/preintegration_odo.h"

PreintegrationOdo::PreintegrationOdo(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0,
                                     IntegrationState state)
    : PreintegrationBase(std::move(parameters), imu0, std::move(state)) {

    // Reset state
    resetState(state, NUM_STATE);

    // Set initial noise matrix
    setNoiseMatrix();

    // 里程计参数
    cvb_  = Rotation::euler2matrix(parameters_->abv).transpose();
    lodo_ = parameters_->lodo;
}

Eigen::MatrixXd PreintegrationOdo::evaluate(const IntegrationState &state0, const IntegrationState &state1,
                                            double *residuals) {
    sqrt_information_ =
        Eigen::LLT<Eigen::Matrix<double, NUM_STATE, NUM_STATE>>(covariance_.inverse()).matrixL().transpose();

    Eigen::Map<Eigen::Matrix<double, NUM_STATE, 1>> residual(residuals);

    Matrix3d dp_dbg   = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba   = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg   = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba   = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg   = jacobian_.block<3, 3>(6, 9);
    Vector3d ds_dsodo = jacobian_.block<3, 1>(15, 18);
    Matrix3d ds_dbg   = jacobian_.block<3, 3>(15, 9);

    // 零偏误差
    Vector3d dbg = state0.bg - delta_state_.bg;
    Vector3d dba = state0.ba - delta_state_.ba;
    double dsodo = state0.sodo - delta_state_.sodo;

    // 积分校正
    corrected_p_ = delta_state_.p + dp_dba * dba + dp_dbg * dbg;
    corrected_v_ = delta_state_.v + dv_dba * dba + dv_dbg * dbg;
    corrected_q_ = delta_state_.q * Rotation::rotvec2quaternion(dq_dbg * dbg);
    corrected_s_ = delta_state_.s + ds_dbg * dbg + ds_dsodo * dsodo;

    // Residuals
    residual.block<3, 1>(0, 0) = state0.q.inverse() * (state1.p - state0.p - state0.v * delta_time_ -
                                                       0.5 * gravity_ * delta_time_ * delta_time_) -
                                 corrected_p_;
    residual.block<3, 1>(3, 0)  = state0.q.inverse() * (state1.v - state0.v - gravity_ * delta_time_) - corrected_v_;
    residual.block<3, 1>(6, 0)  = 2 * (corrected_q_.inverse() * state0.q.inverse() * state1.q).vec();
    residual.block<3, 1>(9, 0)  = state1.bg - state0.bg;
    residual.block<3, 1>(12, 0) = state1.ba - state0.ba;
    residual.block<3, 1>(15, 0) = state0.q.inverse() * (state1.p - state0.p) - corrected_s_;
    residual(18)                = state1.sodo - state0.sodo;

    residual = sqrt_information_ * residual;
    return residual;
}

Eigen::MatrixXd PreintegrationOdo::residualJacobianPose0(const IntegrationState &state0, const IntegrationState &state1,
                                                         double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(0, 0, 3, 3) = -state0.q.inverse().toRotationMatrix();
    jaco.block(0, 3, 3, 3) =
        Rotation::skewSymmetric(state0.q.inverse() * (state1.p - state0.p - state0.v * delta_time_ -
                                                      0.5 * gravity_ * delta_time_ * delta_time_));
    jaco.block(3, 3, 3, 3) =
        Rotation::skewSymmetric(state0.q.inverse() * (state1.v - state0.v - gravity_ * delta_time_));
    jaco.block(6, 3, 3, 3) =
        -(Rotation::quaternionleft(state1.q.inverse() * state0.q) * Rotation::quaternionright(corrected_q_))
             .bottomRightCorner<3, 3>();
    jaco.block(15, 0, 3, 3) = -state0.q.inverse().toRotationMatrix();
    jaco.block(15, 3, 3, 3) = Rotation::skewSymmetric(state0.q.inverse() * (state1.p - state0.p));

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd PreintegrationOdo::residualJacobianPose1(const IntegrationState &state0, const IntegrationState &state1,
                                                         double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(0, 0, 3, 3) = state0.q.inverse().toRotationMatrix();
    jaco.block(6, 3, 3, 3) =
        Rotation::quaternionleft(corrected_q_.inverse() * state0.q.inverse() * state1.q).bottomRightCorner<3, 3>();
    jaco.block(15, 0, 3, 3) = state0.q.inverse().toRotationMatrix();

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd PreintegrationOdo::residualJacobianMix0(const IntegrationState &state0, const IntegrationState &state1,
                                                        double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Matrix3d dp_dbg   = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba   = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg   = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba   = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg   = jacobian_.block<3, 3>(6, 9);
    Vector3d ds_dsodo = jacobian_.block<3, 1>(15, 18);
    Matrix3d ds_dbg   = jacobian_.block<3, 3>(15, 9);

    jaco.block(0, 0, 3, 3) = -state0.q.inverse().toRotationMatrix() * delta_time_;
    jaco.block(0, 3, 3, 3) = -dp_dbg;
    jaco.block(0, 6, 3, 3) = -dp_dba;
    jaco.block(3, 0, 3, 3) = -state0.q.inverse().toRotationMatrix();
    jaco.block(3, 3, 3, 3) = -dv_dbg;
    jaco.block(3, 6, 3, 3) = -dv_dba;
    jaco.block(6, 3, 3, 3) =
        -Rotation::quaternionleft(state1.q.inverse() * state0.q * delta_state_.q).bottomRightCorner<3, 3>() * dq_dbg;
    jaco.block(9, 3, 3, 3)  = -Eigen::Matrix3d::Identity();
    jaco.block(12, 6, 3, 3) = -Eigen::Matrix3d::Identity();
    jaco.block(15, 3, 3, 3) = -ds_dbg;
    jaco.block(15, 9, 3, 1) = -ds_dsodo;
    jaco(18, 9)             = -1.0;

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd PreintegrationOdo::residualJacobianMix1(const IntegrationState &state0, const IntegrationState &state1,
                                                        double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(3, 0, 3, 3)  = state0.q.inverse().toRotationMatrix();
    jaco.block(9, 3, 3, 3)  = Eigen::Matrix3d::Identity();
    jaco.block(12, 6, 3, 3) = Eigen::Matrix3d::Identity();
    jaco(18, 9)             = 1.0;

    jaco = sqrt_information_ * jaco;
    return jaco;
}

int PreintegrationOdo::numResiduals() {
    return NUM_STATE;
}

vector<int> PreintegrationOdo::numBlocksParameters() {
    return vector<int>{NUM_POSE, NUM_MIX, NUM_POSE, NUM_MIX};
}

IntegrationStateData PreintegrationOdo::stateToData(const IntegrationState &state) {
    IntegrationStateData data;
    PreintegrationBase::stateToData(state, data);
    data.mix[9] = state.sodo;

    return data;
}

IntegrationState PreintegrationOdo::stateFromData(const IntegrationStateData &data) {
    IntegrationState state;
    PreintegrationBase::stateFromData(data, state);
    state.sodo = data.mix[9];

    return state;
}

void PreintegrationOdo::constructState(const double *const *parameters, IntegrationState &state0,
                                       IntegrationState &state1) {
    state0 = IntegrationState{
        .p    = {parameters[0][0], parameters[0][1], parameters[0][2]},
        .q    = {parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]},
        .v    = {parameters[1][0], parameters[1][1], parameters[1][2]},
        .bg   = {parameters[1][3], parameters[1][4], parameters[1][5]},
        .ba   = {parameters[1][6], parameters[1][7], parameters[1][8]},
        .sodo = parameters[1][9],
    };

    state1 = IntegrationState{
        .p    = {parameters[2][0], parameters[2][1], parameters[2][2]},
        .q    = {parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]},
        .v    = {parameters[3][0], parameters[3][1], parameters[3][2]},
        .bg   = {parameters[3][3], parameters[3][4], parameters[3][5]},
        .ba   = {parameters[3][6], parameters[3][7], parameters[3][8]},
        .sodo = parameters[3][9],
    };
}

void PreintegrationOdo::integrationProcess(unsigned long index) {
    IMU imu_pre = compensationBias(imu_buffer_[index - 1]);
    IMU imu_cur = compensationBias(imu_buffer_[index]);

    // 连续状态积分和预积分
    // 相对里程预积分
    Vector3d dsodo = Vector3d(imu_cur.odovel, 0, 0);
    delta_state_.s += delta_state_.q.toRotationMatrix() *
                      (cvb_ * dsodo * (1 + delta_state_.sodo) -
                       Rotation::rotvec2quaternion(imu_cur.dtheta).toRotationMatrix() * lodo_ + lodo_);
    integration(imu_pre, imu_cur);

    // 更新系统状态雅克比和协方差矩阵
    updateJacobianAndCovariance(imu_pre, imu_cur);
}

void PreintegrationOdo::resetState(const IntegrationState &state) {
    resetState(state, NUM_STATE);
}

void PreintegrationOdo::updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) {
    // dp, dv, dq, dbg, dba, ds, dsodo

    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(NUM_STATE, NUM_STATE);

    double dt = imu_cur.dt;

    // jacobian

    // phi = I + F * dt
    phi.block<3, 3>(0, 0)   = Matrix3d::Identity();
    phi.block<3, 3>(0, 3)   = Matrix3d::Identity() * dt;
    phi.block<3, 3>(3, 3)   = Matrix3d::Identity();
    phi.block<3, 3>(3, 6)   = -delta_state_.q.toRotationMatrix() * Rotation::skewSymmetric(imu_cur.dvel);
    phi.block<3, 3>(3, 12)  = -delta_state_.q.toRotationMatrix() * dt;
    phi.block<3, 3>(6, 6)   = Matrix3d::Identity() - Rotation::skewSymmetric(imu_cur.dtheta);
    phi.block<3, 3>(6, 9)   = -Matrix3d::Identity() * dt;
    phi.block<3, 3>(9, 9)   = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);
    phi.block<3, 3>(12, 12) = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);

    Vector3d dsodo  = Vector3d(imu_cur.odovel, 0, 0);
    Vector3d stheta = cvb_ * dsodo * (1 + delta_state_.sodo) - imu_cur.dtheta.cross(lodo_);

    phi.block<3, 3>(15, 6)  = -delta_state_.q.toRotationMatrix() * Rotation::skewSymmetric(stheta);
    phi.block<3, 3>(15, 9)  = -delta_state_.q.toRotationMatrix() * Rotation::skewSymmetric(lodo_) * dt;
    phi.block<3, 3>(15, 15) = Matrix3d::Identity();
    phi.block<3, 1>(15, 18) = delta_state_.q.toRotationMatrix() * cvb_ * dsodo;
    phi(18, 18)             = 1.0;

    jacobian_ = phi * jacobian_;

    // covariance

    Eigen::MatrixXd gt = Eigen::MatrixXd::Zero(NUM_STATE, NUM_NOISE);

    gt.block<3, 3>(3, 3)   = delta_state_.q.toRotationMatrix();
    gt.block<3, 3>(6, 0)   = Matrix3d::Identity();
    gt.block<3, 3>(9, 6)   = Matrix3d::Identity();
    gt.block<3, 3>(12, 9)  = Matrix3d::Identity();
    gt.block<3, 3>(15, 0)  = delta_state_.q.toRotationMatrix() * Rotation::skewSymmetric(lodo_);
    gt.block<3, 3>(15, 12) = delta_state_.q.toRotationMatrix() * cvb_ * (1 + delta_state_.sodo);
    gt(18, 15)             = 1.0;

    Eigen::MatrixXd Qk =
        0.5 * dt * (phi * gt * noise_ * gt.transpose() + gt * noise_ * gt.transpose() * phi.transpose());
    covariance_ = phi * covariance_ * phi.transpose() + Qk;
}

void PreintegrationOdo::resetState(const IntegrationState &state, int num) {

    delta_time_ = 0;
    delta_state_.p.setZero();
    delta_state_.q.setIdentity();
    delta_state_.v.setZero();
    delta_state_.s.setZero();
    delta_state_.bg   = state.bg;
    delta_state_.ba   = state.ba;
    delta_state_.sodo = state.sodo;

    jacobian_.setIdentity(num, num);
    covariance_.setZero(num, num);
}

void PreintegrationOdo::setNoiseMatrix() {
    noise_.setIdentity(NUM_NOISE, NUM_NOISE);
    noise_.block<3, 3>(0, 0) *= parameters_->gyr_arw * parameters_->gyr_arw; // nw
    noise_.block<3, 3>(3, 3) *= parameters_->acc_vrw * parameters_->acc_vrw; // na
    noise_.block<3, 3>(6, 6) *=
        2 * parameters_->gyr_bias_std * parameters_->gyr_bias_std / parameters_->corr_time; // nbg
    noise_.block<3, 3>(9, 9) *=
        2 * parameters_->acc_bias_std * parameters_->acc_bias_std / parameters_->corr_time; // nba
    noise_(12, 12) *= parameters_->odo_std[0] * parameters_->odo_std[0];                    // nodo
    noise_(13, 13) *= parameters_->odo_std[1] * parameters_->odo_std[1];                    // nodo
    noise_(14, 14) *= parameters_->odo_std[2] * parameters_->odo_std[2];                    // nodo
    noise_(15, 15) *= parameters_->odo_srw * parameters_->odo_srw;                          // nsodo
}

int PreintegrationOdo::numMixParametersBlocks() {
    return NUM_MIX;
}
