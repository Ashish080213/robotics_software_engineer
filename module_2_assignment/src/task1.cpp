#include <chrono>
#include <functional>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(double radius)
        : Node("drive_circle"), radius(radius)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));

        linear_velocity = 0.5; // Constant Linear Speed
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = linear_velocity;
        message.angular.z = linear_velocity / radius;
        RCLCPP_INFO(this->get_logger(), "Driving Robot with speed: %f, angular velocity: %f and radius: %f",
                    message.linear.x, message.angular.z, radius);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    double radius;          // Radius of the Circle
    double linear_velocity; // Linear Velocity
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Radius input 
    double radius;
    std::cout << "Enter the radius of the circle (>0 in meters): ";
    std::cin >> radius;

    if (radius <= 0)
    {
        std::cerr << "Radius must be greater than zero!" << std::endl;
        return 1;
    }

    rclcpp::spin(std::make_shared<MinimalPublisher>(radius));
    rclcpp::shutdown();
    return 0;
}