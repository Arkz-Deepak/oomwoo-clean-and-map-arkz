#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

struct GridPoint {
    int x;
    int y;
};

class CoveragePlanner : public rclcpp::Node
{
public:
    CoveragePlanner() : Node("coverage_planner")
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        RCLCPP_INFO(this->get_logger(), "Coverage Planner Node Initialized!");
        
        // Initialize the Action Client for Nav2
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
            this, "navigate_through_poses");

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/coverage_path", 10);
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CoveragePlanner::mapCallback, this, std::placeholders::_1));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Prevent the node from re-sending the path every time the map updates
        if (path_sent_) return; 

        path_waypoints_.clear();
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        
        double sweep_width = 0.35; 
        int y_step = std::max(1, static_cast<int>(sweep_width / resolution));

        double safety_margin = 0.35;
        int margin_cells = static_cast<int>(safety_margin / resolution);

        auto is_point_safe = [&](int px, int py) {
            for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
                for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
                    int nx = px + dx;
                    int ny = py + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int idx = ny * width + nx;
                        if (msg->data[idx] != 0) return false;
                    }
                }
            }
            return true;
        };

        // 1. Generate ONLY the extreme corner waypoints for each row
        for (int y = margin_cells; y < height - margin_cells; y += y_step) {
            int sweep_row = (y - margin_cells) / y_step;
            bool moving_right = (sweep_row % 2 == 0);
            
            int start_x = moving_right ? margin_cells : width - margin_cells - 1;
            int end_x = moving_right ? width - margin_cells : margin_cells - 1;
            int step = moving_right ? 1 : -1;

            int first_valid_x = -1;
            int last_valid_x = -1;

            // Scan the row to find the start and end of the free space
            for (int x = start_x; x != end_x; x += step) {
                if (is_point_safe(x, y)) {
                    if (first_valid_x == -1) first_valid_x = x;
                    last_valid_x = x;
                }
            }

            // Only add the endpoints of the row!
            if (first_valid_x != -1 && last_valid_x != -1) {
                path_waypoints_.push_back({first_valid_x, y});
                if (first_valid_x != last_valid_x) {
                    path_waypoints_.push_back({last_valid_x, y});
                }
            }
        }

        // 2. Lock in the frame and time directly
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;

        for (const auto& point : path_waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            
            // Strictly hardcode to "map" and force the current time!
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            
            pose.pose.position.x = origin_x + (point.x + 0.5) * resolution;
            pose.pose.position.y = origin_y + (point.y + 0.5) * resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0; 
            
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Generated and published %zu corner waypoints!", path_msg.poses.size());

        if (!path_msg.poses.empty()) {
            sendNavGoal(path_msg.poses);
            path_sent_ = true;
        }
    }

    void sendNavGoal(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
    {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action server not available! Is Navigation running?");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
        goal_msg.poses = poses;

        RCLCPP_INFO(this->get_logger(), "Handing path over to Nav2. The robot should start moving...");
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Success! The robot completed the coverage path.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Nav2 aborted or canceled the coverage path.");
            }
        };

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav_client_;
    std::vector<GridPoint> path_waypoints_;
    bool path_sent_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoveragePlanner>());
    rclcpp::shutdown();
    return 0;
}