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
    : Node("task4_node")
    {
      this->declare_parameter<double>("linear_speed", 0.0);
      linear_speed_ = this->get_parameter("linear_speed").as_double();

      this->declare_parameter<double>("angular_speed", 0.25);
      angular_speed_ = this->get_parameter("angular_speed").as_double();
      
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
        message.linear.x = linear_speed_;
        message.angular.z = angular_speed_;
      }
      else if (cmd_vel_topic_ == "/turtle4/cmd_vel") {
        message.linear.x = linear_speed_;
        message.angular.z = angular_speed_;
      }
      else {
        message.linear.x = linear_speed_;
        message.angular.z = angular_speed_;
      }
      RCLCPP_INFO(this->get_logger(), "Driving Robot with speed: %f and angular speed: %f", message.linear.x, message.angular.z);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    std::string cmd_vel_topic_;
    double linear_speed_;
    double angular_speed_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}