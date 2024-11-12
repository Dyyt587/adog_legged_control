#ifndef LEG_SOFTWARE_HPP
#define LGE_SOFTWARE_HPP
//实际：线速度跟右前腿的角速度方向相反

#include <Eigen/Eigen>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

// const double hip=0.0672;
// const double thigh=0.15;
// const double calf=0.16;
const double hip=0.0955;
const double thigh=0.213;
const double calf=0.223;

const double pi=M_PI;

#define hip_max 1.57f // 胯关节限幅
#define hip_min -1.57f
#define thigh_max 1.57f // 大腿限幅
#define thigh_min -1.57f
#define calf_max 0.0f // 小腿限幅
#define calf_min -2.7227f

#define base2hip_offset_x  0.1934   //基坐标到hip坐标的偏移
#define base2hip_offset_y  0.0465
#define base2hip_offset_z  0.0


typedef class Leg
{
    private:
        Eigen::Matrix3Xf m;             //雅可比矩阵

        void matrix_count(void);
        void inverse_calculate(const Eigen::Vector3d coordinate);
        
    public:

        typedef struct{
            Eigen::Vector3d angle;      //关节角度gamma alpha beta 
            Eigen::Vector3d wspeed;     //关节线速度
            Eigen::Vector3d tor;        //关节力矩
            Eigen::Vector3d k_pos;      //关节位置系数
            Eigen::Vector3d k_speed;    //关节速度系数
        }joint_state_t;     //关节状态

        typedef struct{
            Eigen::Vector3d coordinate;  //足端坐标xyz
            Eigen::Vector3d lspeed;      //足端线速度
            Eigen::Vector3d force;       //足端力
        }foot_state_t;      //足端状态

        double l1=0;
        double l2=0;
        double l3=0;

        Eigen::Vector3d base2hip_offset;    //base到hip的关节偏移

        joint_state_t joint_present;      //关节实时反馈状态
        joint_state_t joint_target;       //关节期望状态

        foot_state_t foot_present;       //足端实时反馈计算状态
        foot_state_t foot_target;        //足端期望状态

        Leg()   //构造函数
        {
            Eigen::Matrix3Xf a(3, 3);
            m = a;
            // l1 = hip;
            // l2 = -thigh;
            // l3 = -calf;
        }
        Eigen::Vector3d angle_inverse_1;    //运动学逆解
        Eigen::Vector3d angle_inverse_2; 
        Eigen::Vector3d angle_inverse_3; 
        Eigen::Vector3d angle_inverse_4; 

        void jointwspeed2foot(Eigen::Vector3d wspeed,Eigen::Vector3d &lspeed);
        void jointtor2foot(Eigen::Vector3d tor,Eigen::Vector3d &force);
        void footspeed2joint(Eigen::Vector3d &wspeed,Eigen::Vector3d lspeed);
        void footforce2joint(Eigen::Vector3d &tor,Eigen::Vector3d force);

        void correct_calculate(Eigen::Vector3d &coordinate,Eigen::Vector3d angle);
        int get_best_inverse(Eigen::Vector3d coordinate,Eigen::Vector3d &angle);

        double count_gamma_1(const Eigen::Vector3d coordinate);
        double count_gamma_2(const Eigen::Vector3d coordinate);
        double count_alpha(const Eigen::Vector3d coordinate,double gamma_count, double beta_count);
        double count_beta_1(const Eigen::Vector3d coordinate);
        double count_beta_2(const Eigen::Vector3d coordinate);

        // void set_data(USB &usb,joint_state_t &data);
        // void get_data(USB &usb,joint_state_t &data);
        // void leg_test(USB &usb);
}Leg;


#endif


