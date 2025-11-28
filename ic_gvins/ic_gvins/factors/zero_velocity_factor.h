// ic_gvins/factors/zero_velocity_factor.h

#include <ceres/ceres.h>
#include "preintegration/preintegration.h"

// 這是 "一元因子" (Unary Factor)，只依賴於 mix 變量
class ZuptVelocityFactor : public ceres::CostFunction {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // velocity_std: 零速觀測的標準差，越小代表越相信靜止
    ZuptVelocityFactor(double velocity_std, 
                       const Preintegration::PreintegrationOptions& options) 
        : sqrt_info_(1.0 / velocity_std) {
        
        // 設置殘差維度：3 (Vx, Vy, Vz)
        set_num_residuals(3);
        
        // 設置參數塊維度：Mix Block (包含 Vel, Bg, Ba...)
        // 我們需要知道 Mix Block 的大小，這取決於是否有 Odometer 等
        int mix_dim = Preintegration::numMixParameter(options);
        mutable_parameter_block_sizes()->push_back(mix_dim);
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const override {
        // parameters[0] 是 mix 參數塊
        // GVINS 中 mix 的佈局: 
        // [0-2]: Vel, [3-5]: Bg, [6-8]: Ba, ...
        
        const double* mix = parameters[0];
        const double vx = mix[0];
        const double vy = mix[1];
        const double vz = mix[2];

        // 殘差 r = (v - 0) * weight
        residuals[0] = vx * sqrt_info_;
        residuals[1] = vy * sqrt_info_;
        residuals[2] = vz * sqrt_info_;

        if (jacobians && jacobians[0]) {
            // 雅可比矩陣 J = d(residual) / d(mix)
            // 是一個 3 x mix_dim 的矩陣
            int mix_dim = parameter_block_sizes()[0];
            Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> jacobian_matrix(jacobians[0], 3, mix_dim);
            
            jacobian_matrix.setZero();

            // 只對 Velocity 部分 (前三列) 有導數
            jacobian_matrix(0, 0) = sqrt_info_;
            jacobian_matrix(1, 1) = sqrt_info_;
            jacobian_matrix(2, 2) = sqrt_info_;
        }

        return true;
    }

private:
    double sqrt_info_;
};