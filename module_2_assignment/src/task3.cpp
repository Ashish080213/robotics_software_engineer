#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("task3_node")
    {
      this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
      cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      if (cmd_vel_topic_ == "/turtle3/cmd_vel") {
        message.linear.x = 0.25;
        message.angular.z = -0.25;
      }
      else if (cmd_vel_topic_ == "/turtle4/cmd_vel") {
        message.linear.x = 0.25;
        message.angular.z = 0.25;
      }
      else {
        message.linear.x = 0.0;
        message.angular.z = 0.25;
      }
      RCLCPP_INFO(this->get_logger(), "Driving Robot with speed: %f and angular speed: %f", message.linear.x, message.angular.z);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::string cmd_vel_topic_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}