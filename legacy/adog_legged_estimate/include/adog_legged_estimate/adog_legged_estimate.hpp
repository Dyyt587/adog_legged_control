/*
 * @Author: Dyyt587 805207319@qq.com
 * @Date: 2024-10-03 16:22:05
 * @LastEditors: Dyyt587 805207319@qq.com
 * @LastEditTime: 2024-10-05 19:52:40
 * @FilePath: /test_ws/adog_estimate/include/adog_estimate/kalaman.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
/**
 * 卡尔曼公式:
 *状态方程  X(k) = A * X(k-1) +B * U(k-1) + W(k-1)
 *观测方程  Z(k) = H * X_(k)  + V(k)
 X(k) 是对应k时刻的状态变量，
 U(k-1) 是对应k-1时刻的输入，A是状态转移矩阵，B是控制矩阵，
 W(k-1) 是对应k-1时刻的过程噪声，满足一个N(0,Q)的正态分布，其中Q是正态分布的协方差矩阵。

 Z(k) 是对应k时刻的测量值，H是测量矩阵，
 V(k) 是对应k时刻的测量噪声，满足一个N(0,R)的正态分布，其中R是对应的协方差矩阵。

 先验估计 X^^(k) = A * X^(k-1) +B * U(k-1)
 先验误差协方差 P^(k) = A(k) * P(k-1) * A_T(k) + Q
 卡尔曼增益 K(k) = ( P^(k)*H_T ) / ( H * P^(k)*H_T + R)
 后验估计 X^(k) = X^^(k) + K(k) * (Z(k) - H*x^^(k))

 更新误差协方差矩阵 P(k) = (I-K(k)*H)*P^(k)

 */
#include <Eigen/Dense>
#include <iostream>
class EKF_Estimate
{
public:
    /**
     * Create a Kalman filter with the specified matrices.
     *   A - System dynamics matrix
     *   Q - Process noise covariance
     *   R - Measurement noise covariance
     *   P - Estimate error covariance
     */
    EKF_Estimate(
        const Eigen::MatrixXd &A,
        const Eigen::MatrixXd &Q,
        const Eigen::MatrixXd &R,
        const Eigen::MatrixXd &P)
    {
        int r = A.rows();
        int c = A.cols();

        X_Est.resize(c);
        std::cout << "X_Est:" << std::endl
                  << X_Est << std::endl;
        _Q = Q;
        _R = R;
        _P = P;
        // TODO: 判断行列数是否合规
        std::cout << "A:" << std::endl
                  << A << std::endl;
    }
    void update(const Eigen::MatrixXd A, const Eigen::MatrixXd B, const Eigen::MatrixXd u)
    {
        // 记得计算dt
        X_Est = A * X_Est + (B * u);
        _P = A * _P * A.transpose() + _Q;
        std::cout << "X_Est:" << std::endl
                  << X_Est << std::endl;

        std::cout << "A:" << std::endl
                  << A << std::endl;
        std::cout << "_P:" << std::endl
                  << _P << std::endl;
                  
    };

private:
    Eigen::MatrixXd _P; // 协方差
    Eigen::MatrixXd H;
    Eigen::VectorXd X;
    Eigen::VectorXd X_Est;
    Eigen::MatrixXd B;
    Eigen::MatrixXd u;
    Eigen::MatrixXd Z;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _R;
};