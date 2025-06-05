#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include "algo_astar.h"
#include "algo_rrt.hpp"

class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning() : Node("path_planning_node"), use_astar_(false),
                     use_rrt_(true)
    {
        RCLCPP_INFO(this->get_logger(), "Path Planning Node Initialized");

        occupancy_grid_subscriber_ =
            this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10,
                std::bind(&PathPlanning::occupancyGridCallback, this,
                          std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // A* SET-1
        // start_point_.position.x = -8.77246;
        // start_point_.position.y = 7.73288;
        // goal_point_.position.x = 7.83457;
        // goal_point_.position.y = -5.8982;
        // A* SET-2
        // start_point_.position.x = -7.66468;
        // start_point_.position.y = -6.49703;
        // goal_point_.position.x = 7.91067;
        // goal_point_.position.y = 6.5931;
        // A* SET-3
        start_point_.position.x = -8.77246;
        start_point_.position.y = 7.73288;
        goal_point_.position.x = 7.91067;
        goal_point_.position.y = 6.5931;

        // RRT using Occupancy Grid
        // start_point_.position.x = -3.5;
        // start_point_.position.y = -2.0;
        // goal_point_.position.x = 0.5;
        // goal_point_.position.y = -1.5;
        
        // RRT SET-1
        start_point_.position.x = 5.0;
        start_point_.position.y = 5.0;
        goal_point_.position.x = 0.0;
        goal_point_.position.y = 0.0;
    }

private:
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid)
    {
        RCLCPP_INFO(this->get_logger(), "Received occupancy grid");

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = grid.header.frame_id;
        path_msg.header.stamp = this->get_clock()->now();

        if (use_astar_)
        {
            path_msg.poses = astar_search(grid, start_point_, goal_point_);
            RCLCPP_INFO(this->get_logger(), "A* planner executed");
        }

        else if (use_rrt_)
        {
            RRT_Planner planner;
            std::vector<int> domain(grid.data.begin(), grid.data.end());
            RCLCPP_INFO(this->get_logger(), "RRT planner executed");
            planner.setDomain(domain);

            int start_x = (start_point_.position.x - grid.info.origin.position.x) / grid.info.resolution;
            int start_y = (start_point_.position.y - grid.info.origin.position.y) / grid.info.resolution;
            int goal_x = (goal_point_.position.x - grid.info.origin.position.x) / grid.info.resolution;
            int goal_y = (goal_point_.position.y - grid.info.origin.position.y) / grid.info.resolution;

            std::cout << "Converted Start point: (" << start_x << ", " << start_y << ")" << std::endl;
            std::cout << "Converted Goal point: (" << goal_x << ", " << goal_y << ")" << std::endl;

            NodeRrt start(start_x, start_y);
            NodeRrt goal(goal_x, goal_y);

            RCLCPP_INFO(this->get_logger(), "Started Planning");
            std::vector<NodeRrt> path = planner.planPath(start, goal);

            RCLCPP_INFO(this->get_logger(), "Done Planning");

            if (path.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No path found by RRT planner");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Path found by RRT planner");
                for (const auto &node : path)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = node.getX() * grid.info.resolution + grid.info.origin.position.x;
                    pose.pose.position.y = node.getY() * grid.info.resolution + grid.info.origin.position.y;
                    pose.pose.position.z = 0.0;
                    path_msg.poses.push_back(pose);
                }
            }
            RCLCPP_INFO(this->get_logger(), "RRT planner executed");
        }

        RCLCPP_INFO(this->get_logger(), "Publishing Path");
        path_publisher_->publish(path_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    geometry_msgs::msg::Pose start_point_;
    geometry_msgs::msg::Pose goal_point_;
    bool use_astar_;
    bool use_rrt_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
