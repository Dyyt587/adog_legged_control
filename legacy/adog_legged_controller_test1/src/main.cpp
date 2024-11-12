#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "adog_legged_interfaces/msg/multi_joint_mit.hpp"
#include "adog_legged_interfaces/msg/joint_mit.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "adog_legged_controller_test/leg_software.hpp"
#include <ruckig/ruckig.hpp>
#include "nav_msgs/msg/path.hpp"
/**
 *
 *     _____________
 *  FL |   head    | FR
 *     |           |
 *     |           |
 *     |           |
 *     |           |
 *     |           |
 *     |           |
 *  RL ————————————— RR
 *
 *
 *
 *
 *
 */

enum
{
	FL = 0, // 左前腿
	FR,
	RL,
	RR,
	LEG_NUM
};

const double speed_x = 0.01;
const double speed_y = 0.01;
const double speed_z = 0.01;

const double speed_ax = 0.01;
const double speed_ay = 0.01;
const double speed_az = 0.01;

const double speed_jx = 0.01;
const double speed_jy = 0.01;
const double speed_jz = 0.01;

const double pos_start_x = 0;
const double pos_start_y = 0.0955;
const double pos_start_z = -0.1;

const double pos_end_x = 0;
const double pos_end_y = 0.0955;
const double pos_end_z = -0.3;

double k_speed_hip = 0;
double k_speed_thigh = 0;
double k_speed_calf = 0;

double k_pos_hip = 0;
double k_pos_thigh = 0;
double k_pos_calf = 0;

double k_speed_hip_l = 0;
double k_speed_thigh_l = 0;
double k_speed_calf_l = 0;

double k_pos_hip_l = 0;
double k_pos_thigh_l = 0;
double k_pos_calf_l = 0;

using namespace std::chrono_literals;
using namespace ruckig;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
std::vector<Leg> leg_vector(LEG_NUM); // 定义leg变量容器
Leg &FL_leg = leg_vector[FL];
Leg &FR_leg = leg_vector[FR];
Leg &RL_leg = leg_vector[RL];
Leg &RR_leg = leg_vector[RR];

std::vector<ruckig::Ruckig<3>> otg_vector(4); // control cycle
std::vector<ruckig::InputParameter<3>> input_vector(4);
std::vector<ruckig::OutputParameter<3>> output_vector(4);

std::vector<nav_msgs::msg::Path> path_vector(4);

class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher()
		: Node("minimal_publisher"), count_(0)
	{

		FL_leg.l1 = hip;
		FL_leg.l2 = -thigh;
		FL_leg.l3 = -calf;

		FR_leg.l1 = -hip;
		FR_leg.l2 = -thigh;
		FR_leg.l3 = -calf;

		RL_leg.l1 = hip;
		RL_leg.l2 = -thigh;
		RL_leg.l3 = -calf;

		RR_leg.l1 = -hip;
		RR_leg.l2 = -thigh;
		RR_leg.l3 = -calf;

		FL_leg.base2hip_offset[0] = base2hip_offset_x;
		FL_leg.base2hip_offset[1] = base2hip_offset_y;
		FL_leg.base2hip_offset[2] = base2hip_offset_z;

		FR_leg.base2hip_offset[0] = base2hip_offset_x;
		FR_leg.base2hip_offset[1] = -base2hip_offset_y;
		FR_leg.base2hip_offset[2] = base2hip_offset_z;

		RL_leg.base2hip_offset[0] = -base2hip_offset_x;
		RL_leg.base2hip_offset[1] = base2hip_offset_y;
		RL_leg.base2hip_offset[2] = base2hip_offset_z;

		RR_leg.base2hip_offset[0] = -base2hip_offset_x;
		RR_leg.base2hip_offset[1] = -base2hip_offset_y;
		RR_leg.base2hip_offset[2] = base2hip_offset_z;

		otg_vector[0].delta_time = 0.01;
		otg_vector[1].delta_time = 0.01;
		otg_vector[2].delta_time = 0.01;
		otg_vector[3].delta_time = 0.01;

		path_vector[0].header.frame_id = "trunk";
		path_vector[1].header.frame_id = "trunk";
		path_vector[2].header.frame_id = "trunk";
		path_vector[3].header.frame_id = "trunk";

		// Set input parameters
		input_vector[0].current_position = {pos_start_x + FL_leg.base2hip_offset[0], pos_start_y + FL_leg.base2hip_offset[1], pos_start_z + FL_leg.base2hip_offset[2]};
		input_vector[0].current_velocity = {0.0, 0, 0};
		input_vector[0].current_acceleration = {0.0, 0, 0};

		input_vector[0].target_position = {pos_end_x + FL_leg.base2hip_offset[0], pos_end_y + FL_leg.base2hip_offset[1], pos_end_z + FL_leg.base2hip_offset[2]};
		input_vector[0].target_velocity = {0.0, 0, 0};
		input_vector[0].target_acceleration = {0.0, 0.0, 0};

		input_vector[0].max_velocity = {speed_x, speed_y, speed_z};
		input_vector[0].max_acceleration = {speed_ax, speed_ay, speed_az};
		input_vector[0].max_jerk = {speed_jx, speed_jy, speed_jz};

		input_vector[1].current_position = {pos_start_x + FR_leg.base2hip_offset[0], -pos_start_y + FR_leg.base2hip_offset[1], pos_start_z + FR_leg.base2hip_offset[2]};
		input_vector[1].current_velocity = {0.0, 0, 0};
		input_vector[1].current_acceleration = {0.0, 0, 0};

		input_vector[1].target_position = {pos_end_x + FR_leg.base2hip_offset[0], -pos_end_y + FR_leg.base2hip_offset[1], pos_end_z + FR_leg.base2hip_offset[2]};
		input_vector[1].target_velocity = {0.0, 0, 0};
		input_vector[1].target_acceleration = {0.0, 0.0, 0};

		input_vector[1].max_velocity = {speed_x, speed_y, speed_z};
		input_vector[1].max_acceleration = {speed_ax, speed_ay, speed_az};
		input_vector[1].max_jerk = {speed_jx, speed_jy, speed_jz};

		input_vector[2].current_position = {pos_start_x + RL_leg.base2hip_offset[0], pos_start_y + RL_leg.base2hip_offset[1], pos_start_z + RL_leg.base2hip_offset[2]};
		input_vector[2].current_velocity = {0.0, 0, 0};
		input_vector[2].current_acceleration = {0.0, 0, 0};

		input_vector[2].target_position = {pos_end_x + RL_leg.base2hip_offset[0], pos_end_y + RL_leg.base2hip_offset[1], pos_end_z + RL_leg.base2hip_offset[2]};
		input_vector[2].target_velocity = {0.0, 0, 0};
		input_vector[2].target_acceleration = {0.0, 0.0, 0};

		input_vector[2].max_velocity = {speed_x, speed_y, speed_z};
		input_vector[2].max_acceleration = {speed_ax, speed_ay, speed_az};
		input_vector[2].max_jerk = {speed_jx, speed_jy, speed_jz};

		input_vector[3].current_position = {pos_start_x + RR_leg.base2hip_offset[0], -pos_start_y + RR_leg.base2hip_offset[1], pos_start_z + RR_leg.base2hip_offset[2]};
		input_vector[3].current_velocity = {0.0, 0, 0};
		input_vector[3].current_acceleration = {0.0, 0, 0};

		input_vector[3].target_position = {pos_end_x + RR_leg.base2hip_offset[0], -pos_end_y + RR_leg.base2hip_offset[1], pos_end_z + RR_leg.base2hip_offset[2]};
		input_vector[3].target_velocity = {0.0, 0, 0};
		input_vector[3].target_acceleration = {0.0, 0.0, 0};

		input_vector[3].max_velocity = {speed_x, speed_y, speed_z};
		input_vector[3].max_acceleration = {speed_ax, speed_ay, speed_az};
		input_vector[3].max_jerk = {speed_jx, speed_jy, speed_jz};

		// publisher_ = this->create_publisher< adog_legged_interfaces::msg::MultiJointMit>("/adog_legged_controller/reference", 10);
		publisher_ = this->create_publisher<adog_legged_interfaces::msg::MultiJointMit>("/joint_effort_controller/reference", 10);
		track_fl_publisher_ = this->create_publisher<nav_msgs::msg::Path>("joint_fl_track", 10);
		track_fr_publisher_ = this->create_publisher<nav_msgs::msg::Path>("joint_fr_track", 10);
		track_rl_publisher_ = this->create_publisher<nav_msgs::msg::Path>("joint_rl_track", 10);
		track_rr_publisher_ = this->create_publisher<nav_msgs::msg::Path>("joint_rr_track", 10);
		subscribe_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&MinimalPublisher::subscribe_callback, this, std::placeholders::_1));
		// subscribe_track = this->create_subscription<nav_msgs::msg::Path>("/joint_track", 10, std::bind(&MinimalPublisher::subtrack_callback, this, std::placeholders::_1));
		timer_ = this->create_wall_timer(5ms, std::bind(&MinimalPublisher::timer_callback, this));
	}

private:
	uint8_t sequence_table[12]; // 顺序表，用来对返回值进行排序（返回顺序可能错误）
	void JointStateToLeg(const sensor_msgs::msg::JointState &msg)
	{
		for (size_t i = 0; i < 3; ++i)
		{
			FL_leg.joint_present.angle(i) = msg.position[sequence_table[i]];
			FL_leg.joint_present.wspeed(i) = msg.velocity[sequence_table[i]];
			FL_leg.joint_present.tor(i) = msg.effort[sequence_table[i]];
		}

		for (size_t i = 0; i < 3; ++i)
		{
			FR_leg.joint_present.angle(i) = msg.position[sequence_table[i + 3]];
			FR_leg.joint_present.wspeed(i) = msg.velocity[sequence_table[i + 3]];
			FR_leg.joint_present.tor(i) = msg.effort[sequence_table[i + 3]];
		}
		for (size_t i = 0; i < 3; ++i)
		{
			RL_leg.joint_present.angle(i) = msg.position[sequence_table[i + 6]];
			RL_leg.joint_present.wspeed(i) = msg.velocity[sequence_table[i + 6]];
			RL_leg.joint_present.tor(i) = msg.effort[sequence_table[i + 6]];
		}
		for (size_t i = 0; i < 3; ++i)
		{
			RR_leg.joint_present.angle(i) = msg.position[sequence_table[i + 9]];
			RR_leg.joint_present.wspeed(i) = msg.velocity[sequence_table[i + 9]];
			RR_leg.joint_present.tor(i) = msg.effort[sequence_table[i + 9]];
		}
	}
	void subscribe_callback(sensor_msgs::msg::JointState msg)
	{
		static uint8_t number = 0;
		if (number <= 10)
		{ // 接收第10次时，去寻找顺序
			number++;
			if (number == 10)
			{
				uint8_t match_number = 0;
				const char match_sequence[12][15] = {
					"FL_hip_joint",
					"FL_thigh_joint",
					"FL_calf_joint",
					"FR_hip_joint",
					"FR_thigh_joint",
					"FR_calf_joint",
					"RL_hip_joint",
					"RL_thigh_joint",
					"RL_calf_joint",
					"RR_hip_joint",
					"RR_thigh_joint",
					"RR_calf_joint",
				};
				for (int i = 0; i < 12; i++)
				{
					for (int j = 0; j < 12; j++)
					{
						size_t found = msg.name[j].find(match_sequence[i]);
						if (found == 0)
						{
							// RCLCPP_INFO(this->get_logger(), "22squence:%d,%d",i,j);
							match_number++;
							sequence_table[i] = j;
							break;
						}
					}
				}
				if (match_number != 12)
				{
					for (int i = 0; i < 10; i++)
					{
						RCLCPP_INFO(this->get_logger(), "sub_match error:%d", match_number);
						//_exit(-1);
					}
				}
				// std::cout<<sequence_table<<std::endl;
			}
		}
		else
		{
			JointStateToLeg(msg);
		}
	}
	void timer_callback()
	{
		auto message = adog_legged_interfaces::msg::MultiJointMit();
		message.multi_jointmit_array.assign(12, adog_legged_interfaces::msg::JointMit());

		static int count = 0;
		static int start_flag = 0;
		static int total_number = 0;

		if (count % 2 == 0 && start_flag == 1)
		{
			input_vector[0].current_position = {pos_start_x + FL_leg.base2hip_offset[0], pos_start_y + FL_leg.base2hip_offset[1], pos_start_z + FL_leg.base2hip_offset[2]};
			input_vector[0].target_position = {pos_end_x + FL_leg.base2hip_offset[0], pos_end_y + FL_leg.base2hip_offset[1], pos_end_z + FL_leg.base2hip_offset[2]};
			path_vector[0].poses.clear();

			input_vector[1].current_position = {pos_start_x + FR_leg.base2hip_offset[0], -pos_start_y + FR_leg.base2hip_offset[1], pos_start_z + FR_leg.base2hip_offset[2]};
			input_vector[1].target_position = {pos_end_x + FR_leg.base2hip_offset[0], -pos_end_y + FR_leg.base2hip_offset[1], pos_end_z + FR_leg.base2hip_offset[2]};
			path_vector[1].poses.clear();

			input_vector[2].current_position = {pos_start_x + RL_leg.base2hip_offset[0], pos_start_y + RL_leg.base2hip_offset[1], pos_start_z + RL_leg.base2hip_offset[2]};
			input_vector[2].target_position = {pos_end_x + RL_leg.base2hip_offset[0], pos_end_y + RL_leg.base2hip_offset[1], pos_end_z + RL_leg.base2hip_offset[2]};
			path_vector[2].poses.clear();

			input_vector[3].current_position = {pos_start_x + RR_leg.base2hip_offset[0], -pos_start_y + RR_leg.base2hip_offset[1], pos_start_z + RR_leg.base2hip_offset[2]};
			input_vector[3].target_position = {pos_end_x + RR_leg.base2hip_offset[0], -pos_end_y + RR_leg.base2hip_offset[1], pos_end_z + RR_leg.base2hip_offset[2]};
			path_vector[3].poses.clear();
			start_flag = 0;

			FL_leg.foot_target.coordinate[0] = pos_end_x + FL_leg.base2hip_offset[0];
			FL_leg.foot_target.coordinate[1] = pos_end_y + FL_leg.base2hip_offset[1];
			FL_leg.foot_target.coordinate[2] = pos_end_z + FL_leg.base2hip_offset[2];
			FL_leg.get_best_inverse(FL_leg.foot_target.coordinate, FL_leg.joint_target.angle);

			FR_leg.foot_target.coordinate[0] = pos_end_x + FR_leg.base2hip_offset[0];
			FR_leg.foot_target.coordinate[1] = -pos_end_y + FR_leg.base2hip_offset[1];
			FR_leg.foot_target.coordinate[2] = pos_end_z + FR_leg.base2hip_offset[2];
			FR_leg.get_best_inverse(FR_leg.foot_target.coordinate, FR_leg.joint_target.angle);

			RL_leg.foot_target.coordinate[0] = pos_end_x + RL_leg.base2hip_offset[0];
			RL_leg.foot_target.coordinate[1] = pos_end_y + RL_leg.base2hip_offset[1];
			RL_leg.foot_target.coordinate[2] = pos_end_z + RL_leg.base2hip_offset[2];
			RL_leg.get_best_inverse(RL_leg.foot_target.coordinate, RL_leg.joint_target.angle);

			RR_leg.foot_target.coordinate[0] = pos_end_x + RR_leg.base2hip_offset[0];
			RR_leg.foot_target.coordinate[1] = -pos_end_y + RR_leg.base2hip_offset[1];
			RR_leg.foot_target.coordinate[2] = pos_end_z + RR_leg.base2hip_offset[2];
			RR_leg.get_best_inverse(RR_leg.foot_target.coordinate, RR_leg.joint_target.angle);

			RCLCPP_INFO(this->get_logger(), "inverse_data:%f,%f,%f", FL_leg.joint_target.angle(0),
						FL_leg.joint_target.angle(1), FL_leg.joint_target.angle(2));
		}
		else if (count % 2 == 1 && start_flag == 1)
		{
			input_vector[0].current_position = {pos_end_x + FL_leg.base2hip_offset[0], pos_end_y + FL_leg.base2hip_offset[1], pos_end_z + FL_leg.base2hip_offset[2]};
			input_vector[0].target_position = {pos_start_x + FL_leg.base2hip_offset[0], pos_start_y + FL_leg.base2hip_offset[1], pos_start_z + FL_leg.base2hip_offset[2]};
			path_vector[0].poses.clear();
			input_vector[1].current_position = {pos_end_x + FR_leg.base2hip_offset[0], -pos_end_y + FR_leg.base2hip_offset[1], pos_end_z + FR_leg.base2hip_offset[2]};
			input_vector[1].target_position = {pos_start_x + FR_leg.base2hip_offset[0], -pos_start_y + FR_leg.base2hip_offset[1], pos_start_z + FR_leg.base2hip_offset[2]};
			path_vector[1].poses.clear();
			input_vector[2].current_position = {pos_end_x + RL_leg.base2hip_offset[0], pos_end_y + RL_leg.base2hip_offset[1], pos_end_z + RL_leg.base2hip_offset[2]};
			input_vector[2].target_position = {pos_start_x + RL_leg.base2hip_offset[0], pos_start_y + RL_leg.base2hip_offset[1], pos_start_z + RL_leg.base2hip_offset[2]};
			path_vector[2].poses.clear();
			input_vector[3].current_position = {pos_end_x + RR_leg.base2hip_offset[0], -pos_end_y + RR_leg.base2hip_offset[1], pos_end_z + RR_leg.base2hip_offset[2]};
			input_vector[3].target_position = {pos_start_x + RR_leg.base2hip_offset[0], -pos_start_y + RR_leg.base2hip_offset[1], pos_start_z + RR_leg.base2hip_offset[2]};
			path_vector[3].poses.clear();
			start_flag = 0;

			FL_leg.foot_target.coordinate[0] = pos_start_x + FL_leg.base2hip_offset[0];
			FL_leg.foot_target.coordinate[1] = pos_start_y + FL_leg.base2hip_offset[1];
			FL_leg.foot_target.coordinate[2] = pos_start_z + FL_leg.base2hip_offset[2];
			FL_leg.get_best_inverse(FL_leg.foot_target.coordinate, FL_leg.joint_target.angle);

			FR_leg.foot_target.coordinate[0] = pos_start_x + FR_leg.base2hip_offset[0];
			FR_leg.foot_target.coordinate[1] = -pos_start_y + FR_leg.base2hip_offset[1];
			FR_leg.foot_target.coordinate[2] = pos_start_z + FR_leg.base2hip_offset[2];
			FR_leg.get_best_inverse(FR_leg.foot_target.coordinate, FR_leg.joint_target.angle);

			RL_leg.foot_target.coordinate[0] = pos_start_x + RL_leg.base2hip_offset[0];
			RL_leg.foot_target.coordinate[1] = pos_start_y + RL_leg.base2hip_offset[1];
			RL_leg.foot_target.coordinate[2] = pos_start_z + RL_leg.base2hip_offset[2];
			RL_leg.get_best_inverse(RL_leg.foot_target.coordinate, RL_leg.joint_target.angle);

			RR_leg.foot_target.coordinate[0] = pos_start_x + RR_leg.base2hip_offset[0];
			RR_leg.foot_target.coordinate[1] = -pos_start_y + RR_leg.base2hip_offset[1];
			RR_leg.foot_target.coordinate[2] = pos_start_z + RR_leg.base2hip_offset[2];
			RR_leg.get_best_inverse(RR_leg.foot_target.coordinate, RR_leg.joint_target.angle);

			RCLCPP_INFO(this->get_logger(), "inverse_data:%f,%f,%f", FL_leg.joint_target.angle(0),
						FL_leg.joint_target.angle(1), FL_leg.joint_target.angle(2));
		}
		if (otg_vector[0].update(input_vector[0], output_vector[0]) == 0)
		{
			total_number++;

			otg_vector[1].update(input_vector[1], output_vector[1]);
			otg_vector[2].update(input_vector[2], output_vector[2]);
			otg_vector[3].update(input_vector[3], output_vector[3]);
			geometry_msgs::msg::PoseStamped pose;
			pose.header.frame_id = "trunk";
			pose.header.stamp = get_clock()->now();

			pose.pose.position.x = output_vector[0].new_position[0];
			pose.pose.position.y = output_vector[0].new_position[1];
			pose.pose.position.z = output_vector[0].new_position[2];
			path_vector[FL].poses.push_back(pose);

			FL_leg.joint_target.wspeed[0] = output_vector[0].new_velocity[0];
			FL_leg.joint_target.wspeed[1] = output_vector[0].new_velocity[1];
			FL_leg.joint_target.wspeed[2] = output_vector[0].new_velocity[2];

			pose.pose.position.x = output_vector[1].new_position[0];
			pose.pose.position.y = output_vector[1].new_position[1];
			pose.pose.position.z = output_vector[1].new_position[2];
			path_vector[FR].poses.push_back(pose);

			FR_leg.joint_target.wspeed[0] = output_vector[1].new_velocity[0];
			FR_leg.joint_target.wspeed[1] = output_vector[1].new_velocity[1];
			FR_leg.joint_target.wspeed[2] = output_vector[1].new_velocity[2];

			pose.pose.position.x = output_vector[2].new_position[0];
			pose.pose.position.y = output_vector[2].new_position[1];
			pose.pose.position.z = output_vector[2].new_position[2];
			path_vector[RL].poses.push_back(pose);

			RL_leg.joint_target.wspeed[0] = output_vector[2].new_velocity[0];
			RL_leg.joint_target.wspeed[1] = output_vector[2].new_velocity[1];
			RL_leg.joint_target.wspeed[2] = output_vector[2].new_velocity[2];

			pose.pose.position.x = output_vector[3].new_position[0];
			pose.pose.position.y = output_vector[3].new_position[1];
			pose.pose.position.z = output_vector[3].new_position[2];
			path_vector[RR].poses.push_back(pose);

			RR_leg.joint_target.wspeed[0] = output_vector[3].new_velocity[0];
			RR_leg.joint_target.wspeed[1] = output_vector[3].new_velocity[1];
			RR_leg.joint_target.wspeed[2] = output_vector[3].new_velocity[2];

			// RCLCPP_INFO(this->get_logger(),"publish:%lf,%lf,%lf",FL_leg.joint_target.angle[0],FL_leg.joint_target.angle[1],FL_leg.joint_target.angle[2]);

			output_vector[0].pass_to_input(input_vector[0]);
			output_vector[1].pass_to_input(input_vector[1]);
			output_vector[2].pass_to_input(input_vector[2]);
			output_vector[3].pass_to_input(input_vector[3]);
			track_fl_publisher_->publish(path_vector[0]);
			track_fr_publisher_->publish(path_vector[1]);
			track_rl_publisher_->publish(path_vector[2]);
			track_rr_publisher_->publish(path_vector[3]);
			// RCLCPP_INFO(this->get_logger(),"publish");
		}
		else
		{
			// track_fl_publisher_->publish(path_vector[0]);
			total_number = 0;
			start_flag = 1;
			count++;
			RCLCPP_INFO(this->get_logger(), "convert");
		}

		FL_leg.joint_target.k_pos(0) = k_pos_hip;
		FL_leg.joint_target.k_pos(1) = k_pos_thigh;
		FL_leg.joint_target.k_pos(2) = k_pos_calf;
		FL_leg.joint_target.k_speed(0) = k_speed_hip;
		FL_leg.joint_target.k_speed(1) = k_speed_thigh;
		FL_leg.joint_target.k_speed(2) = k_speed_calf;

		FR_leg.joint_target.k_pos(0) = k_pos_hip;
		FR_leg.joint_target.k_pos(1) = k_pos_thigh;
		FR_leg.joint_target.k_pos(2) = k_pos_calf;
		FR_leg.joint_target.k_speed(0) = k_speed_hip;
		FR_leg.joint_target.k_speed(1) = k_speed_thigh;
		FR_leg.joint_target.k_speed(2) = k_speed_calf;

		RL_leg.joint_target.k_pos(0) = k_pos_hip_l;
		RL_leg.joint_target.k_pos(1) = k_pos_thigh_l;
		RL_leg.joint_target.k_pos(2) = k_pos_calf_l;
		RL_leg.joint_target.k_speed(0) = k_speed_hip_l;
		RL_leg.joint_target.k_speed(1) = k_speed_thigh_l;
		RL_leg.joint_target.k_speed(2) = k_speed_calf_l;

		RR_leg.joint_target.k_pos(0) = k_pos_hip_l;
		RR_leg.joint_target.k_pos(1) = k_pos_thigh_l;
		RR_leg.joint_target.k_pos(2) = k_pos_calf_l;
		RR_leg.joint_target.k_speed(0) = k_speed_hip_l;
		RR_leg.joint_target.k_speed(1) = k_speed_thigh_l;
		RR_leg.joint_target.k_speed(2) = k_speed_calf_l;
		// front_left_leg
		message.multi_jointmit_array[0].position = FL_leg.joint_target.angle(0);
		message.multi_jointmit_array[0].effort = FL_leg.joint_target.tor(0);
		message.multi_jointmit_array[0].velocity = FL_leg.joint_target.wspeed(0);
		message.multi_jointmit_array[0].kp = FL_leg.joint_target.k_pos(0);
		message.multi_jointmit_array[0].kd = FL_leg.joint_target.k_speed(0);

		message.multi_jointmit_array[1].position = FL_leg.joint_target.angle(1);
		message.multi_jointmit_array[1].effort = FL_leg.joint_target.tor(1);
		message.multi_jointmit_array[1].velocity = FL_leg.joint_target.wspeed(1);
		message.multi_jointmit_array[1].kp = FL_leg.joint_target.k_pos(1);
		message.multi_jointmit_array[1].kd = FL_leg.joint_target.k_speed(1);

		message.multi_jointmit_array[2].position = FL_leg.joint_target.angle(2);
		message.multi_jointmit_array[2].effort = FL_leg.joint_target.tor(2);
		message.multi_jointmit_array[2].velocity = FL_leg.joint_target.wspeed(2);
		message.multi_jointmit_array[2].kp = FL_leg.joint_target.k_pos(2);
		message.multi_jointmit_array[2].kd = FL_leg.joint_target.k_speed(2);
		// front_right_leg
		message.multi_jointmit_array[3].position = FR_leg.joint_target.angle(0);
		message.multi_jointmit_array[3].effort = FR_leg.joint_target.tor(0);
		message.multi_jointmit_array[3].velocity = FR_leg.joint_target.wspeed(0);
		message.multi_jointmit_array[3].kp = FR_leg.joint_target.k_pos(0);
		message.multi_jointmit_array[3].kd = FR_leg.joint_target.k_speed(0);

		message.multi_jointmit_array[4].position = FR_leg.joint_target.angle(1);
		message.multi_jointmit_array[4].effort = FR_leg.joint_target.tor(1);
		message.multi_jointmit_array[4].velocity = FR_leg.joint_target.wspeed(1);
		message.multi_jointmit_array[4].kp = FR_leg.joint_target.k_pos(1);
		message.multi_jointmit_array[4].kd = FR_leg.joint_target.k_speed(1);

		message.multi_jointmit_array[5].position = FR_leg.joint_target.angle(2);
		message.multi_jointmit_array[5].effort = FR_leg.joint_target.tor(2);
		message.multi_jointmit_array[5].velocity = FR_leg.joint_target.wspeed(2);
		message.multi_jointmit_array[5].kp = FR_leg.joint_target.k_pos(2);
		message.multi_jointmit_array[5].kd = FR_leg.joint_target.k_speed(2);
		// late_left_leg
		message.multi_jointmit_array[6].position = RL_leg.joint_target.angle(0);
		message.multi_jointmit_array[6].effort = RL_leg.joint_target.tor(0);
		message.multi_jointmit_array[6].velocity = RL_leg.joint_target.wspeed(0);
		message.multi_jointmit_array[6].kp = RL_leg.joint_target.k_pos(0);
		message.multi_jointmit_array[6].kd = RL_leg.joint_target.k_speed(0);

		message.multi_jointmit_array[7].position = RL_leg.joint_target.angle(1);
		message.multi_jointmit_array[7].effort = RL_leg.joint_target.tor(1);
		message.multi_jointmit_array[7].velocity = RL_leg.joint_target.wspeed(1);
		message.multi_jointmit_array[7].kp = RL_leg.joint_target.k_pos(1);
		message.multi_jointmit_array[7].kd = RL_leg.joint_target.k_speed(1);

		message.multi_jointmit_array[8].position = RL_leg.joint_target.angle(2);
		message.multi_jointmit_array[8].effort = RL_leg.joint_target.tor(2);
		message.multi_jointmit_array[8].velocity = RL_leg.joint_target.wspeed(2);
		message.multi_jointmit_array[8].kp = RL_leg.joint_target.k_pos(2);
		message.multi_jointmit_array[8].kd = RL_leg.joint_target.k_speed(2);
		// late_right_leg
		message.multi_jointmit_array[9].position = RR_leg.joint_target.angle(0);
		message.multi_jointmit_array[9].effort = RR_leg.joint_target.tor(0);
		message.multi_jointmit_array[9].velocity = RR_leg.joint_target.wspeed(0);
		message.multi_jointmit_array[9].kp = RR_leg.joint_target.k_pos(0);
		message.multi_jointmit_array[9].kd = RR_leg.joint_target.k_speed(0);

		message.multi_jointmit_array[10].position = RR_leg.joint_target.angle(1);
		message.multi_jointmit_array[10].effort = RR_leg.joint_target.tor(1);
		message.multi_jointmit_array[10].velocity = RR_leg.joint_target.wspeed(1);
		message.multi_jointmit_array[10].kp = RR_leg.joint_target.k_pos(1);
		message.multi_jointmit_array[10].kd = RR_leg.joint_target.k_speed(1);

		message.multi_jointmit_array[11].position = RR_leg.joint_target.angle(2);
		message.multi_jointmit_array[11].effort = RR_leg.joint_target.tor(2);
		message.multi_jointmit_array[11].velocity = RR_leg.joint_target.wspeed(2);
		message.multi_jointmit_array[11].kp = RR_leg.joint_target.k_pos(2);
		message.multi_jointmit_array[11].kd = RR_leg.joint_target.k_speed(2);
		// publisher_->publish(message);
		// RCLCPP_INFO(this->get_logger(), "Publishing");

		publisher_->publish(message);
	}
	// 声明订阅者，发布者
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<adog_legged_interfaces::msg::MultiJointMit>::SharedPtr publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_fl_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_fr_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_rl_publisher_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_rr_publisher_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscribe_;
	// rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscribe_track;
	size_t count_;
};

int main(int argc, char *argv[])
{

	// Generate the trajectory within the control loop

	// std::cout << "t | position" << std::endl;
	//  while (otg.update(input, output) == ruckig::Result::Working) {
	//      output.new_position
	//      output.pass_to_input(input);
	//  }
	scanf("%lf,%lf,%lf,%lf,%lf,%lf", &k_pos_hip, &k_speed_hip, &k_pos_thigh, &k_speed_thigh, &k_pos_calf, &k_speed_calf);
	scanf("%lf,%lf,%lf,%lf,%lf,%lf", &k_pos_hip_l, &k_speed_hip_l, &k_pos_thigh_l, &k_speed_thigh_l, &k_pos_calf_l, &k_speed_calf_l);
	printf("%lf,%lf,%lf,%lf,%lf,%lf\n", k_pos_hip, k_speed_hip, k_pos_thigh, k_speed_thigh, k_pos_calf, k_speed_calf);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}
