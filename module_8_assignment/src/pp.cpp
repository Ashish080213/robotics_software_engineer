#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include "algo_astar.h"
#include "algo_rrt.hpp"    // RRT_Planner and NodeRrt
#include "algo_rrtstar.hpp" // RRTStar_Planner and NodeRRTStar

class PathPlanning : public rclcpp::Node
{
public:
    PathPlanning() : Node("pp_node"),
                     use_astar_(false),
                     use_rrt_(false),
                     use_rrtstar_(true)  // enable RRT*
    {
        RCLCPP_INFO(this->get_logger(), "Path Planning Node Initialized");

        occupancy_grid_subscriber_ =
            this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10,
                std::bind(&PathPlanning::occupancyGridCallback, this,
                          std::placeholders::_1));

        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // start and goal points
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
            // RRT planner
            RRT_Planner planner;
            std::vector<int> domain(grid.data.begin(), grid.data.end());
            planner.setDomain(domain);

            int start_x = (start_point_.position.x - grid.info.origin.position.x) / grid.info.resolution;
            int start_y = (start_point_.position.y - grid.info.origin.position.y) / grid.info.resolution;
            int goal_x = (goal_point_.position.x - grid.info.origin.position.x) / grid.info.resolution;
            int goal_y = (goal_point_.position.y - grid.info.origin.position.y) / grid.info.resolution;

            NodeRrt start(start_x, start_y);
            NodeRrt goal(goal_x, goal_y);

            RCLCPP_INFO(this->get_logger(), "Started RRT planning");
            std::vector<NodeRrt> path = planner.planPath(start, goal);
            RCLCPP_INFO(this->get_logger(), "Done RRT planning");

            if (path.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No path found by RRT planner");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Path found by RRT planner, length: %zu", path.size());
                for (const auto &node : path)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = node.getX() * grid.info.resolution + grid.info.origin.position.x;
                    pose.pose.position.y = node.getY() * grid.info.resolution + grid.info.origin.position.y;
                    pose.pose.position.z = 0.0;
                    path_msg.poses.push_back(pose);
                }
            }
        }
        else if (use_rrtstar_)
        {
            // RRT* planner
            RRTStar_Planner planner;
            std::vector<int> domain(grid.data.begin(), grid.data.end());
            planner.setDomain(domain);

            int start_x = (start_point_.position.x - grid.info.origin.position.x) / grid.info.resolution;
            int start_y = (start_point_.position.y - grid.info.origin.position.y) / grid.info.resolution;
            int goal_x = (goal_point_.position.x - grid.info.origin.position.x) / grid.info.resolution;
            int goal_y = (goal_point_.position.y - grid.info.origin.position.y) / grid.info.resolution;

            NodeRRTStar start(start_x, start_y);
            NodeRRTStar goal(goal_x, goal_y);

            RCLCPP_INFO(this->get_logger(), "Started RRT* planning");
            std::vector<NodeRRTStar> path = planner.planPath(start, goal);
            RCLCPP_INFO(this->get_logger(), "Done RRT* planning");

            if (path.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No path found by RRT* planner");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Path found by RRT* planner, length: %zu", path.size());
                for (const auto &node : path)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = node.getX() * grid.info.resolution + grid.info.origin.position.x;
                    pose.pose.position.y = node.getY() * grid.info.resolution + grid.info.origin.position.y;
                    pose.pose.position.z = 0.0;
                    path_msg.poses.push_back(pose);
                }
            }
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
    bool use_rrtstar_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanning>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
