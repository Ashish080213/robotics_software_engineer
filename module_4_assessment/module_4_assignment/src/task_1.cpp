#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class LineFollowing : public rclcpp::Node
{
public:
    LineFollowing() : Node("line_follower"), _angularVel(0.3)
    {
        this->declare_parameter<int>("lower_threshold", 200);
        this->declare_parameter<int>("upper_threshold", 250);
        _publisher =
            this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        _subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&LineFollowing::cameraCallback, this, std::placeholders::_1));
    }

private:
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr camera_msg)
    {
        analyzeImage(camera_msg);
        determineMidPoint();
    }

    void analyzeImage(const sensor_msgs::msg::Image::SharedPtr camera_msg)
    {
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(camera_msg, "bgr8");
        cv::Mat grayImage_;
        cv::cvtColor(cvPtr->image, grayImage_, cv::COLOR_BGR2GRAY);
        int upperThreshold_ = this->get_parameter("upper_threshold").as_int();
        int lowerThreshold_ = this->get_parameter("lower_threshold").as_int();
        cv::Canny(grayImage_, _cannyImage, lowerThreshold_, upperThreshold_);
    }

    void determineMidPoint()
    {
        int row_ = 150, column_ = 0;
        cv::Mat roi_ = _cannyImage(cv::Range(row_, row_ + 240), cv::Range(column_, column_ + 640));

        std::vector<int> edge_;
        for (int i = 0; i < 640; ++i)
        {
            if (roi_.at<uchar>(160, i) == 255)
            {
                edge_.push_back(i);
            }
        }

        if (!edge_.empty())
        {
            int midArea_ = edge_.back() - edge_.front();
            _midPoint = edge_.front() + midArea_ / 2;
            _robotMidPoint = 640 / 2;

            calculateError();

            cv::circle(roi_, cv::Point(_midPoint, 160), 2, cv::Scalar(255, 255, 255), -1);
            cv::circle(roi_, cv::Point(_robotMidPoint, 160), 5, cv::Scalar(255, 255, 255), -1);
            cv::imshow("Image", roi_);
            cv::waitKey(1);
        }
    }

    void calculateError()
    {
        double error_ = _robotMidPoint - _midPoint;
        _velocityMsg.linear.x = 0.1;
        _velocityMsg.angular.z = (error_ < 0) ? -_angularVel : _angularVel;
        _publisher->publish(_velocityMsg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;

    double _angularVel;
    geometry_msgs::msg::Twist _velocityMsg;
    cv::Mat _cannyImage;
    int _robotMidPoint;
    int _midPoint;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineFollowing>());
    rclcpp::shutdown();
    return 0;
}
