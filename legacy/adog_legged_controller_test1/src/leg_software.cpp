#include "adog_legged_controller_test/leg_software.hpp"

//求腿的雅克比矩阵
void Leg::matrix_count(void)
{
    m(0, 0) = 0;
    m(0, 1) = l3 * cos(joint_present.angle(1) + joint_present.angle(2)) + l2 * cos(joint_present.angle(1));
    m(0, 2) = l3 * cos(joint_present.angle(1) + joint_present.angle(2));

    m(1, 0) = (-1 * l1 * sin(joint_present.angle(0))) - (l2 * cos(joint_present.angle(0)) * cos(joint_present.angle(1))) - l3 * cos(joint_present.angle(0)) * cos(joint_present.angle(1) + joint_present.angle(2));
    m(1, 1) = (l2 * sin(joint_present.angle(0)) * sin(joint_present.angle(1))) + (l3 * sin(joint_present.angle(0)) * sin(joint_present.angle(1) + joint_present.angle(2)));
    m(1, 2) = l3 * sin(joint_present.angle(0)) * sin(joint_present.angle(1) + joint_present.angle(2));

    m(2, 0) = (l1 * cos(joint_present.angle(0))) - (l2 * sin(joint_present.angle(0)) * cos(joint_present.angle(1))) - (l3 * sin(joint_present.angle(0)) * cos(joint_present.angle(1) + joint_present.angle(2)));
    m(2, 1) = (-1 * l2 * cos(joint_present.angle(0)) * sin(joint_present.angle(1))) - l3 * cos(joint_present.angle(0)) * sin(joint_present.angle(1) + joint_present.angle(2));
    m(2, 2) = -1 * l3 * cos(joint_present.angle(0)) * sin(joint_present.angle(1) + joint_present.angle(2));
}
// 根据关节角速度求足端线速度
void Leg::jointwspeed2foot(Eigen::Vector3d wspeed,Eigen::Vector3d &lspeed) 
{
    matrix_count();
    lspeed(0) = m(0, 0) * wspeed(0) + m(0, 1) * wspeed(1) + m(0, 2) * wspeed(2);
    lspeed(1) = m(1, 0) * wspeed(0) + m(1, 1) * wspeed(1) + m(1, 2) * wspeed(2);
    lspeed(2) = m(2, 0) * wspeed(0) + m(2, 1) * wspeed(1) + m(2, 2) * wspeed(2);
}
//根据足端速度计算关节角速度
void Leg::footspeed2joint(Eigen::Vector3d &wspeed,Eigen::Vector3d lspeed)
{
    matrix_count();
    if (m.determinant() == 0) // 此时雅可比矩阵不可逆
    {
        //std::cout << "matrix cannot inverse,cannot get foot force" << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("Publisher"),"matrix cannot inverse,cannot get foot force");
        return;
    }
    Eigen::Matrix3Xf n(3, 3);
    n = m.inverse();
    wspeed(0)=n(0,0)*lspeed(0)+n(0,1)*lspeed(1)+n(0,2)*lspeed(2);
    wspeed(1)=n(1,0)*lspeed(0)+n(1,1)*lspeed(1)+n(1,2)*lspeed(2);
    wspeed(2)=n(2,0)*lspeed(0)+n(2,1)*lspeed(1)+n(2,2)*lspeed(2);
}
// 根据关节力矩求足端力
void Leg::jointtor2foot(Eigen::Vector3d tor,Eigen::Vector3d &force)
{
    matrix_count();
    if (m.determinant() == 0) // 此时雅可比矩阵不可逆
    {
        //std::cout << "matrix cannot inverse,cannot get foot force" << std::endl;
        RCLCPP_INFO(rclcpp::get_logger("Publisher"),"matrix cannot inverse,cannot get foot force");
        return;
    }
    Eigen::Matrix3Xf n(3, 3);
    n = m.transpose().inverse();
    force(0) = n(0, 0) * tor(0) + n(0, 1) * tor(1) + n(0, 2) * tor(2);
    force(1) = n(1, 0) * tor(0) + n(1, 1) * tor(1) + n(1, 2) * tor(2);
    force(2) = n(2, 0) * tor(0) + n(2, 1) * tor(1) + n(2, 2) * tor(2);
}
//根据足端力计算关节力矩
void Leg::footforce2joint(Eigen::Vector3d &tor,Eigen::Vector3d force)
{
    matrix_count();
    Eigen::Matrix3Xf n(3, 3);
    n=m.transpose();
    tor(0)=n(0,0)*force(0)+n(0,1)*force(1)+n(0,2)*force(2);
    tor(1)=n(1,0)*force(0)+n(1,1)*force(1)+n(1,2)*force(2);
    tor(2)=n(2,0)*force(0)+n(2,1)*force(1)+n(2,2)*force(2);
}

// 正运动学解算(由关节角度求关节坐标)
void Leg::correct_calculate(Eigen::Vector3d &coordinate,Eigen::Vector3d angle) 
{
    coordinate(0) = l3 * sin(angle(1) + angle(2)) + l2 * sin(angle(1));
    coordinate(1) = -1 * l3 * sin(angle(0)) * cos(angle(1) + angle(2)) + l1 * cos(angle(0)) - l2 * cos(angle(1)) * sin(angle(0));
    coordinate(2) = l3 * cos(angle(0)) * cos(angle(1) + angle(2)) + l1 * sin(angle(0)) + l2 * cos(angle(0)) * cos(angle(1));
    
    coordinate(0)+=base2hip_offset[0];
    coordinate(1)+=base2hip_offset[1];
    coordinate(2)+=base2hip_offset[2];
}
// 运动学逆解算出4个解
void Leg::inverse_calculate(Eigen::Vector3d coordinate)
{
    coordinate(0)-=base2hip_offset[0];
    coordinate(1)-=base2hip_offset[1];
    coordinate(2)-=base2hip_offset[2];

    angle_inverse_1(0) = count_gamma_1(coordinate);
    angle_inverse_1(2) = count_beta_1(coordinate);
    angle_inverse_1(1) = count_alpha(coordinate,angle_inverse_1(0), angle_inverse_1(2));

    angle_inverse_2(0) = count_gamma_1(coordinate);
    angle_inverse_2(2) = count_beta_2(coordinate);
    angle_inverse_2(1) = count_alpha(coordinate,angle_inverse_2(0), angle_inverse_2(2));

    angle_inverse_3(0) = count_gamma_2(coordinate);
    angle_inverse_3(2) = count_beta_1(coordinate);
    angle_inverse_3(1) = count_alpha(coordinate,angle_inverse_3(0), angle_inverse_3(2));

    angle_inverse_4(0) = count_gamma_2(coordinate);
    angle_inverse_4(2) = count_beta_2(coordinate);
    angle_inverse_4(1) = count_alpha(coordinate,angle_inverse_4(0), angle_inverse_4(2));
    // RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"angle:%f,%f,%f",angle_inverse_1(0),\
    //         angle_inverse_1(1),angle_inverse_1(2));
    // RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"angle:%f,%f,%f",angle_inverse_2(0),\
    //     angle_inverse_2(1),angle_inverse_2(2));
    // RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"angle:%f,%f,%f",angle_inverse_3(0),\
    //     angle_inverse_3(1),angle_inverse_3(2));
    // RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"angle:%f,%f,%f",angle_inverse_4(0),\
    //     angle_inverse_4(1),angle_inverse_4(2));

            
}
//获取最优逆运动学解(由足端坐标计算出关节角度)
int Leg::get_best_inverse(Eigen::Vector3d coordinate,Eigen::Vector3d &angle)
{
    //RCLCPP_INFO(rclcpp::get_logger("Publisher"),"cordinate:%f,%f,%f",coordinate(0),coordinate(1),coordinate(2));
    Eigen::Vector3d correct_coordinate;

    const double error_value = 0.01;
    inverse_calculate(coordinate);
    correct_calculate(correct_coordinate,angle_inverse_1);
    if (fabs(correct_coordinate(0)-coordinate(0)) < error_value && fabs(correct_coordinate(1)-coordinate(1)) < error_value && fabs(correct_coordinate(2)-coordinate(2)) < error_value)
    {
        if (angle_inverse_1(0) >= hip_min && angle_inverse_1(0) <= hip_max &&
            angle_inverse_1(1) >= thigh_min && angle_inverse_1(1)<= thigh_max &&
            angle_inverse_1(2) >= calf_min && angle_inverse_1(2)<= calf_max)
        {
            angle=angle_inverse_1;
            //RCLCPP_INFO(rclcpp::get_logger("inverse_success"),"success");
            return 1;
        }
        else
        {
            //RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"limit_error:angle:%f,%f,%f",angle_inverse_1(0),\
            angle_inverse_1(1),angle_inverse_1(2));
        }
    }
    correct_calculate(correct_coordinate,angle_inverse_2);
    //RCLCPP_INFO(rclcpp::get_logger("Publisher"),"inverse:%f,%f,%f",angle_inverse_2(0),angle_inverse_2(1),angle_inverse_2(2));
    if (fabs(correct_coordinate(0)-coordinate(0)) < error_value && fabs(correct_coordinate(1)-coordinate(1)) < error_value && fabs(correct_coordinate(2)-coordinate(2)) < error_value)
    {
        if (angle_inverse_2(0) >= hip_min && angle_inverse_2(0) <= hip_max &&
            angle_inverse_2(1) >= thigh_min && angle_inverse_2(1)<= thigh_max &&
            angle_inverse_2(2) >= calf_min && angle_inverse_2(2)<= calf_max)
        {
            //RCLCPP_INFO(rclcpp::get_logger("inverse_success"),"success");
            angle=angle_inverse_2;
            return 2;
        }
        else
        {
            //RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"limit_error:angle:%f,%f,%f",angle_inverse_2(0),\
            angle_inverse_2(1),angle_inverse_2(2));
        }
    }
    correct_calculate(correct_coordinate,angle_inverse_3);
    //RCLCPP_INFO(rclcpp::get_logger("Publisher"),"inverse:%f,%f,%f",angle_inverse_3(0),angle_inverse_3(1),angle_inverse_3(2));
    if (fabs(correct_coordinate(0)-coordinate(0)) < error_value && fabs(correct_coordinate(1)-coordinate(1)) < error_value && fabs(correct_coordinate(2)-coordinate(2)) < error_value)
    {
        if (angle_inverse_3(0) >= hip_min && angle_inverse_3(0) <= hip_max &&
            angle_inverse_3(1) >= thigh_min && angle_inverse_3(1)<= thigh_max &&
            angle_inverse_3(2) >= calf_min && angle_inverse_3(2)<= calf_max)
        {

            //RCLCPP_INFO(rclcpp::get_logger("inverse_success"),"success");
            angle=angle_inverse_3;
            return 3;
        }
        else
        {
            //RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"limit_error:angle:%f,%f,%f",angle_inverse_3(0),\
            angle_inverse_3(1),angle_inverse_3(2));
        }
    }
    correct_calculate(correct_coordinate,angle_inverse_4);
    //RCLCPP_INFO(rclcpp::get_logger("Publisher"),"inverse:%f,%f,%f",angle_inverse_4(0),angle_inverse_4(1),angle_inverse_4(2));
    if (fabs(correct_coordinate(0)-coordinate(0)) < error_value && fabs(correct_coordinate(1)-coordinate(1)) < error_value && fabs(correct_coordinate(2)-coordinate(2)) < error_value)
    {
        if (angle_inverse_4(0) >= hip_min && angle_inverse_4(0) <= hip_max &&
            angle_inverse_4(1) >= thigh_min && angle_inverse_4(1)<= thigh_max &&
            angle_inverse_4(2) >= calf_min && angle_inverse_4(2)<= calf_max)
        {
            //RCLCPP_INFO(rclcpp::get_logger("inverse_success"),"success");
            angle=angle_inverse_4;
            return 4;
        }
        else
        {
            //RCLCPP_INFO(rclcpp::get_logger("inverse_error"),"limit_error:angle:%f,%f,%f",angle_inverse_4(0),\
            angle_inverse_4(1),angle_inverse_4(2));
        }
    }
    //RCLCPP_INFO(rclcpp::get_logger("inserve error"),"error_input:%f,%f,%f",coordinate(0),coordinate(1),coordinate(2));
    //std::cout<<"inverse error"<<std::endl;
    //_exit(-1);
    return -1;
}

double Leg::count_gamma_1(const Eigen::Vector3d coordinate)
{
    double L = sqrt(fabs(coordinate(1) * coordinate(1) + coordinate(2) * coordinate(2)) - (l1 * l1));
    double denominator = coordinate(1) * l1 - coordinate(2) * L;
    if (denominator == 0.0)
    {
        return pi / 2.0;
    }
    double molecule = coordinate(2) * l1 + coordinate(1) * L;
    return atan2(molecule, denominator);
}
double Leg::count_gamma_2(const Eigen::Vector3d coordinate)
{
    double L = sqrt(fabs(coordinate(1) *coordinate(1) + coordinate(2) * coordinate(2)) - (l1 * l1));

    double denominator = coordinate(1) * l1 + coordinate(2) * L;
    if (denominator == 0.0)
    {
        return pi / 2.0;
    }
    double molecule = coordinate(2) * l1 - coordinate(1) * L;
    return atan2(molecule, denominator);
}

// 在此之前需要先计算gamma和beta
double Leg::count_alpha(const Eigen::Vector3d coordinate,double gamma_count, double beta_count)
{
    double a1, a2, m1, m2;
    a1 = coordinate(1) * sin(gamma_count) - coordinate(2) * cos(gamma_count);
    a2 = coordinate(0);
    m1 = l3 * sin(beta_count);
    m2 = l3 * cos(beta_count) + l2;
    double denominator = a2 * m1 - a1 * m2;
    if (denominator == 0.0)
    {
        return pi / 2.0;
    }
    double molecule = a1 * m1 + a2 * m2;
    return atan2(molecule, denominator);
}
double Leg::count_beta_1(const Eigen::Vector3d coordinate)
{
    double molecule = (l1 * l1) + (l2 * l2) + (l3 * l3) - (coordinate(0) * coordinate(0)) - (coordinate(1) *coordinate(1)) - (coordinate(2) * coordinate(2)); // 分子
    double denominator = 2 * l2 * l3;                                                  // 分母
    if (molecule / denominator > 1)
    {
        return pi - acos(1);
    }
    if (molecule / denominator < -1)
    {
        return pi - acos(-1);
    }
    else
    {
        return pi - acos(molecule / denominator);
    }

}

double Leg::count_beta_2(const Eigen::Vector3d coordinate)
{
    double molecule = (l1 * l1) + (l2 * l2) + (l3 * l3) - (coordinate(0) * coordinate(0)) - (coordinate(1) * coordinate(1)) - (coordinate(2) * coordinate(2)); // 分子
    double denominator = 2 * l2 * l3;                                                  // 分母
    if (molecule / denominator > 1)
    {
        return -pi + acos(1);
    }
    if (molecule / denominator < -1)
    {
        return -pi + acos(-1);
    }
    else
    {
        return -pi + acos(molecule / denominator);
    }
}