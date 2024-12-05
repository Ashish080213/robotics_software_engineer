#include "ekf_lib.hpp"
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 4, 5> Matrix45d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 5, 5> Matrix5d;

class ExtendedKalmanFilter_Node : public rclcpp::Node {
public:
    ExtendedKalmanFilter_Node() : Node("tb3_ekf_node") {
        setMatrices();

        // Subscribe to /odom for position and orientation
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ExtendedKalmanFilter_Node::odomCallback, this, std::placeholders::_1));

        // Subscribe to /imu for IMU data
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ExtendedKalmanFilter_Node::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg = msg;
        if (odom_msg) {
            estimation();
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg = msg;
    }

    void estimation() {
        RCLCPP_INFO(this->get_logger(), "--------> Starting EKF Iterations");

        // EKF Prediction Step
        ekf.predict();
        RCLCPP_INFO(this->get_logger(), "Prediction step completed.");

        // Extract odometry data (x, y, yaw)
        double x = odom_msg->pose.pose.position.x;
        double y = odom_msg->pose.pose.position.y;

        // IMU data (linear acceleration and angular velocity)
        double ax = imu_msg->linear_acceleration.x;
        double az = imu_msg->angular_velocity.z;

        // Set measurements: [x, y, ax, az]
        measurements << x, y, ax, az;

        RCLCPP_INFO(this->get_logger(),
                    "Measurements: x = %f, y = %f, ax = %f, az = %f", 
                    measurements[0], measurements[1], measurements[2], measurements[3]);

        // EKF Update Step
        std::vector<double> cov_values = {cov_odom, cov_odom, cov_imu, cov_imu};
        ekf.updateR(cov_values);
        ekf.update(measurements);
        RCLCPP_INFO(this->get_logger(), "Update step completed.");
        RCLCPP_INFO(this->get_logger(),
                    "Updated: x = %f, y = %f", 
                    ekf.x_[0], ekf.x_[1]);
    }

    void setMatrices() {
        P_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

        F_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

        H_in.setZero();
        R_in.setZero();

        Q_in << 0.1, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0,
                0, 0, 0.1, 0, 0,
                0, 0, 0, 0.1, 0,
                0, 0, 0, 0, 0.1;

        ekf.dt = 0.1;

        ekf.x_pred_ = x_in;
        ekf.z_pred_ = Vector4d::Zero();

        ekf.initialize(x_in, P_in, F_in, H_in, R_in, Q_in);
    }

    Vector5d x_in;
    Matrix5d P_in;
    Matrix5d F_in;
    Matrix45d H_in;
    Matrix4d R_in;
    Matrix5d Q_in;

    Vector4d measurements;

    double cov_imu = 0.012727922061358;
    double cov_odom = 0.028452340807104;

    ExtendedKalmanFilter ekf;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    sensor_msgs::msg::Imu::SharedPtr imu_msg;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtendedKalmanFilter_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
