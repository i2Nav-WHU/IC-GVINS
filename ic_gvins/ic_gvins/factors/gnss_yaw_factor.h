#ifndef GVINS_GNSS_YAW_FACTOR_H
#define GVINS_GNSS_YAW_FACTOR_H

#include "ic_gvins/common/types.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

class GnssYawFactor {
public:
    // P_i (7維: txyz + qxyzw)
    GnssYawFactor(const Vector3d &gnss_vel, double yaw_std)
        : gnss_vel_(gnss_vel),
          sqrt_info_(yaw_std > 0.0 ? 1.0 / yaw_std : 1.0) {}

    template <typename T>
    bool operator()(const T *const pose_i, T *residuals) const {
        // pose_i: {t_i_w, q_w_i}

        T t_i[3];
        T q_w_i[4];
        
        // 1. 提取 IMU 姿態 (q_w_i)
        // Note: Ceres parameter block order is usually txyz then qxyzw
        t_i[0] = pose_i[0]; t_i[1] = pose_i[1]; t_i[2] = pose_i[2];
        q_w_i[0] = pose_i[3]; q_w_i[1] = pose_i[4]; q_w_i[2] = pose_i[5]; q_w_i[3] = pose_i[6]; // qxyzw

        // 2. 將 GNSS 速度 (在 ENU/World 坐標系下) 轉換為 Ceres T 類型
        T V_gnss_w[3];
        V_gnss_w[0] = T(gnss_vel_.x());
        V_gnss_w[1] = T(gnss_vel_.y());
        V_gnss_w[2] = T(gnss_vel_.z());

        // 3. 從 q_w_i 提取 Euler 角 (Roll, Pitch, Yaw)
        T R_w_i[9];
        ceres::QuaternionToRotation(q_w_i, R_w_i);
        
        // 提取 Yaw (通常是繞 Z 軸的旋轉)
        // Yaw = atan2(R_w_i(1, 0), R_w_i(0, 0)) in ZYX convention (for ENU/local horizontal plane)
        // Note: For R_w_i, R_w_i(0,0) is R_00, R_w_i(1,0) is R_10
        T sin_yaw = R_w_i[3]; // R_10
        T cos_yaw = R_w_i[0]; // R_00
        T yaw_imu = atan2(sin_yaw, cos_yaw); 

        // 4. 計算 GNSS 速度的航跡角 (Course Over Ground, COG)
        // COG_Yaw = atan2(V_North, V_East) 
        // 假設您的 ENU 座標系為: X=East, Y=North, Z=Up (或 Z=Down)
        // 這裡 GNSS 速度通常在 ENU 座標系下 (X=East, Y=North)
        T yaw_cog = atan2(V_gnss_w[1], V_gnss_w[0]); // atan2(V_N, V_E)

        // 5. 計算殘差 (確保角度在 [-pi, pi] 內)
        T angle_diff = yaw_imu - yaw_cog;
        
        // 角度規整化 (Normalized angle difference)
        T PI = T(M_PI);
        while (angle_diff > PI) angle_diff -= T(2.0) * PI;
        while (angle_diff < -PI) angle_diff += T(2.0) * PI;

        // 6. 加權殘差 (只有一個自由度: Yaw)
        residuals[0] = angle_diff * T(sqrt_info_);

        return true;
    }

    static ceres::CostFunction *Create(const Vector3d &gnss_vel, double yaw_std) {
        // 1 個殘差 (Yaw)
        // 1 個參數塊: Pose (7維)
        return new ceres::AutoDiffCostFunction<GnssYawFactor, 1, 7>(
            new GnssYawFactor(gnss_vel, yaw_std));
    }

private:
    const Vector3d gnss_vel_;
    const double sqrt_info_;
};

#endif // GVINS_GNSS_YAW_FACTOR_H