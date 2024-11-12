#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "adog_legged_interfaces/msg/multi_joint_mit.hpp"
#include "adog_legged_interfaces/msg/joint_mit.hpp" 
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher< adog_legged_interfaces::msg::MultiJointMit>("/joint_effort_controller/reference", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = adog_legged_interfaces::msg::MultiJointMit();
      message.multi_jointmit_array.assign(12,adog_legged_interfaces::msg::JointMit());
         message.multi_jointmit_array[0].position = 0.0;
      //publisher_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Publishing: ");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<adog_legged_interfaces::msg::MultiJointMit>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
  }
