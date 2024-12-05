#include "task3_lib.hpp"
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "tf2/LinearMath/Quaternion.h"

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

        // Subscribers
        odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ExtendedKalmanFilter_Node::odomCallback, this, std::placeholders::_1));
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ExtendedKalmanFilter_Node::imuCallback, this, std::placeholders::_1));

        fused_pose_pub = this->create_publisher<visualization_msgs::msg::Marker>("fused_pose_marker", 10);
        imu_pose_pub = this->create_publisher<visualization_msgs::msg::Marker>("imu_pose_marker", 10);
        odom_pose_pub = this->create_publisher<visualization_msgs::msg::Marker>("odom_pose_marker", 10);
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

        // Extract odometry data (x, y)
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
                    "Updated: x = %f, y = %f, theta = %f", 
                    ekf.x_[0], ekf.x_[1], ekf.x_[4]);

        visualizeMarkers();
    }

    void visualizeMarkers() {

        tf2::Quaternion q;
        q.setRPY(0, 0, ekf.x_[4]); 
        geometry_msgs::msg::Quaternion quat_msg;
        quat_msg.x = q.x();
        quat_msg.y = q.y();
        quat_msg.z = q.z();
        quat_msg.w = q.w();

        publishMarker(fused_pose_pub, ekf.x_[0], ekf.x_[1], 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.1); //fused data Blue

        publishMarker(odom_pose_pub, odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.1); //odom Green

        publishMarker(imu_pose_pub, imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.1); //IMU Red

    }

    void publishMarker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& publisher,
                                                double x, double y, double z, double dx, double dy, double dz, double dw,
                                                double r, double g, double b, double scale) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "odom";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "tb3_ekf";    
        static int marker_id = 0;
        marker.id = marker_id++;
        // marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = dx;
        marker.pose.orientation.y = dy;
        marker.pose.orientation.z = dz;
        marker.pose.orientation.w = dw;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        publisher->publish(marker);
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

        // SET-1

        // Q_in << 10, 0, 0, 0, 0,
        //         0, 10, 0, 0, 0,
        //         0, 0, 10, 0, 0,
        //         0, 0, 0, 10, 0,
        //         0, 0, 0, 0, 10;

        // SET-2

        // Q_in << 0.1, 0, 0, 0, 0,
        //         0, 0.1, 0, 0, 0,
        //         0, 0, 0.1, 0, 0,
        //         0, 0, 0, 0.1, 0,
        //         0, 0, 0, 0, 0.1;

        // SET-3

        Q_in << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1;

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

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fused_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr odom_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr imu_pose_pub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtendedKalmanFilter_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
