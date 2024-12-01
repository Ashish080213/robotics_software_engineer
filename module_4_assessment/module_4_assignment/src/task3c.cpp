#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

class RobotMetrics : public rclcpp::Node {
public:
  RobotMetrics() : Node("robot_metrics"), _lastLogTime1(this->now()),  _lastLogTime2(this->now()), _logInterval(0.5) {
    _odomSubscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&RobotMetrics::odomCallback, this, std::placeholders::_1));
    _imuSubscription = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&RobotMetrics::imuCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
    double linearVel = odomMsg->twist.twist.linear.x;

    auto currentTime1 = this->now();
    if ((currentTime1 - _lastLogTime1).seconds() >= _logInterval) {
      RCLCPP_INFO(this->get_logger(), "Linear Velocity: %.2f m/s", linearVel);
      _lastLogTime1 = currentTime1;
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg) {
    double linearAccX = imuMsg->linear_acceleration.x;

    auto currentTime2 = this->now();
    if ((currentTime2 - _lastLogTime2).seconds() >= _logInterval) {
      RCLCPP_INFO(this->get_logger(), "Linear Acceleration: %.2f m/s²", linearAccX);
      _lastLogTime2 = currentTime2;
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscription;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imuSubscription;

  rclcpp::Time _lastLogTime1, _lastLogTime2;
  double _logInterval; 
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMetrics>());
  rclcpp::shutdown();
  return 0;
}
