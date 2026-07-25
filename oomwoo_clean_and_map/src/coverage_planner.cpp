#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class CoveragePlanner : public rclcpp::Node
{
public:
    CoveragePlanner() : Node("coverage_planner")
    {
        RCLCPP_INFO(this->get_logger(), "Coverage Planner Node Initialized!");
        
        // Subscriber to read the 2D grid map
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CoveragePlanner::mapCallback, this, std::placeholders::_1));

        // Publisher to send velocity commands to the robot
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/coverage_path", 10);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // This triggers every time the map updates
        RCLCPP_INFO(this->get_logger(), "Map received. Resolution: %f, Width: %d, Height: %d", 
                    msg->info.resolution, msg->info.width, msg->info.height);
                    
        // Clear any old path
        path_waypoints_.clear();

        int width = msg->info.width;
        int height = msg->info.height;

        // Boustrophedon Traversal
        for (int y = 0; y < height; ++y) {
            // Even rows go left-to-right, odd rows go right-to-left
            bool moving_right = (y % 2 == 0);

            int start_x = moving_right ? 0 : width - 1;
            int end_x = moving_right ? width : -1;
            int step = moving_right ? 1 : -1;

            for (int x = start_x; x != end_x; x += step) {
                int index = y * width + x;
                
                // In ROS 2: 0 = Free space, 100 = Obstacle, -1 = Unknown
                if (msg->data[index] == 0) {
                    path_waypoints_.push_back({x, y});
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu Boustrophedon waypoints!", path_waypoints_.size());
        // Initialize the Path message
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        // Extract map metadata
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        // Convert every grid point into a real-world Pose
        for (const auto& point : path_waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();

            // Math: Origin + (Grid Index * Resolution)
            // Adding 0.5 puts the waypoint perfectly in the center of the grid cell
            pose.pose.position.x = origin_x + (point.x + 0.5) * resolution;
            pose.pose.position.y = origin_y + (point.y + 0.5) * resolution;
            pose.pose.position.z = 0.0;
            
            // Default flat orientation
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0; 

            path_msg.poses.push_back(pose);
        }

        // Publish the generated path line to the ROS 2 network
        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published physical path to /coverage_path!");

    }
    struct GridPoint {
        int x;
        int y;
    };
    std::vector<GridPoint> path_waypoints_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoveragePlanner>());
    rclcpp::shutdown();
    return 0;
}