#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "adog_legged_interfaces/msg/multi_joint_mit.hpp"
#include "adog_legged_interfaces/msg/joint_mit.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("gazebo_controller_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_effort_controller/commands", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    // 创建一个TransformListener对象
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void getTransform()
  {
    // 等待直到我们有从"world"到"robot_base"的转换
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      // 从tf_buffer_中获取转换
      transformStamped = tf_buffer_->lookupTransform("base", "FL_foot", rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)  
    {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      rclcpp::shutdown();
      return;
    }

    // 输出转换结果
    RCLCPP_INFO(this->get_logger(), "Received transform: %s", transformStamped);
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float64MultiArray();
    message.data.assign(12, 1);
    getTransform();

 
     publisher_->publish(message);
    //RCLCPP_INFO(this->get_logger(), "Publishing: ");
    //publisher_->publish(message);
  }
 
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

 