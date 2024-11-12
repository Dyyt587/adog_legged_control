/*
 * @Author: Dyyt587 805207319@qq.com
 * @Date: 2024-10-03 16:21:47
 * @LastEditors: Dyyt587 805207319@qq.com
 * @LastEditTime: 2024-10-05 19:48:01
 * @FilePath: /test_ws/adog_estimate/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%A
 *
 */
#include "adog_legged_estimate.hpp"
#include "iostream"

using namespace std;
using namespace Eigen;
int main(void)
{
    Eigen::MatrixXd A(36,36);
    Eigen::MatrixXd Q(36,36);
    Eigen::MatrixXd R(36,36);
    Eigen::MatrixXd P(36,36);

    Eigen::MatrixXd B(36,3);
    Eigen::MatrixXd u(3,1);

    A= MatrixXd::Identity(36,36);
     A.block(0,3,3,3) = MatrixXd::Identity(3,3);
    // A.block(3,3,3,3) = MatrixXd::Identity(3,3);
    
    
    EKF_Estimate est(A, Q, R, P);

     A.block(0,3,3,3) = MatrixXd::Identity(3,3)*2;
     B.block(3,0,3,3) = MatrixXd::Identity(3,3)*2;
     u << 1, 0, 0;
    est.update(A,B,u);
    //cout << A << endl;
}