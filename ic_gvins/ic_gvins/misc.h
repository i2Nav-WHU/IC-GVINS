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

#ifndef GVINS_MISC_H
#define GVINS_MISC_H

#include "common/types.h"
#include "fileio/filesaver.h"

#include "preintegration/integration_state.h"
#include "preintegration/preintegration.h"

#include <deque>
#include <vector>

using std::vector;

class MISC {

public:
    static size_t getInsWindowIndex(const std::deque<std::pair<IMU, IntegrationState>> &window, double time);
    static bool getCameraPoseFromInsWindow(const std::deque<std::pair<IMU, IntegrationState>> &window,
                                           const Pose &pose_b_c, double time, Pose &pose);
    static void statePoseInterpolation(const IntegrationState &state0, const IntegrationState &state1, double midtime,
                                       IntegrationState &state);

    static Pose stateToCameraPose(const IntegrationState &state, const Pose &pose_b_c);
    static Eigen::Matrix4d pose2Twc(const Pose &pose);

    static bool isTheSameTimeNode(double time0, double time1, double interval);
    static size_t getStateDataIndex(const std::deque<double> &timelist, double time, double interval);

    static void insMechanization(const IntegrationConfiguration &config, const IMU &imu_pre, const IMU &imu_cur,
                                 IntegrationState &state);
    static void redoInsMechanization(const IntegrationConfiguration &config, const IntegrationState &updated_state,
                                     size_t reserved_ins_num,
                                     std::deque<std::pair<IMU, IntegrationState>> &ins_windows);

    static int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
    static void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

    static bool getImuSeriesFromTo(const std::deque<std::pair<IMU, IntegrationState>> &ins_windows, double start,
                                   double end, vector<IMU> &series);

    static bool detectZeroVelocity(const vector<IMU> &imu_buffer, double imudatarate, vector<double> &average);

    static void writeNavResult(const IntegrationConfiguration &config, const IntegrationState &state,
                               const FileSaver::Ptr &navfile, const FileSaver::Ptr &errfile,
                               const FileSaver::Ptr &trajfile);

public:
    // 允许的时间对齐误差
    static constexpr double MINIMUM_TIME_INTERVAL = 0.0001;

    // 零速阈值, rad/s, m/s^2
    static constexpr double ZERO_VELOCITY_GYR_THRESHOLD = 0.002;
    static constexpr double ZERO_VELOCITY_ACC_THRESHOLD = 0.1;
};

#endif // GVINS_MISC_H
