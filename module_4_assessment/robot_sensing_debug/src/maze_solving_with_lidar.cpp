#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

enum class RobotState {
  MOVING_STRAIGHT,
  TURNING_LEFT,
  TURNING_RIGHT,
  OUT_OF_MAZE
};

class MazeSolving : public rclcpp::Node {
public:
  MazeSolving() : Node("maze_solver"), state_(RobotState::MOVING_STRAIGHT) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&MazeSolving::lidarCallback, this, std::placeholders::_1));
  }

private:
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidarMsg) {
    float rightObstacle = *std::max_element(lidarMsg->ranges.begin() + 260,
                                            lidarMsg->ranges.begin() + 280);
    float frontObstacle = *std::max_element(lidarMsg->ranges.begin() + 340,
                                            lidarMsg->ranges.begin() + 360);
    float leftObstacle = *std::max_element(lidarMsg->ranges.begin() + 80,
                                           lidarMsg->ranges.begin() + 100);

    RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f, Left: %f",
                frontObstacle, rightObstacle, leftObstacle);

    if (frontObstacle > _frontThreshold && rightObstacle > _frontThreshold &&
        leftObstacle > _frontThreshold) {
      state_ = RobotState::OUT_OF_MAZE;
    } else if (frontObstacle < _frontThreshold) {
      state_ = leftObstacle < rightObstacle ? RobotState::TURNING_RIGHT
                                            : RobotState::TURNING_LEFT;
    }
    else if (frontObstacle > _frontThreshold) {
      state_ = RobotState::MOVING_STRAIGHT;
    }

    geometry_msgs::msg::Twist command;
    switch (state_) {
    case RobotState::MOVING_STRAIGHT:
      command.linear.x = _linearVel;
      command.angular.z = 0.0;
      break;
    case RobotState::TURNING_LEFT:
      command.linear.x = 0.0;
      command.angular.z = _angularVel;
      break;
    case RobotState::TURNING_RIGHT:
      command.linear.x = 0.0;
      command.angular.z = -_angularVel;
      break;
    case RobotState::OUT_OF_MAZE:
      command.linear.x = 0.0;
      command.angular.z = 0.0;
      break;
    }

    publisher_->publish(command);
  }

  float _frontThreshold = 1.5f;
  float _angularVel = 0.5f;
  float _linearVel = 0.5f;
  RobotState state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeSolving>());
  rclcpp::shutdown();
  return 0;
}

