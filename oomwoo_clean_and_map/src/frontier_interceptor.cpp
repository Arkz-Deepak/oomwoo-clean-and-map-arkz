#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class FrontierInterceptor : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    FrontierInterceptor() : Node("frontier_interceptor")
    {
        // Force the node to sync with Gazebo time
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // 1. Host the Action Server disguise
        action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "intercepted_navigate_to_pose",
            std::bind(&FrontierInterceptor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FrontierInterceptor::handle_cancel, this, std::placeholders::_1),
            std::bind(&FrontierInterceptor::handle_accepted, this, std::placeholders::_1)
        );

        // 2. Client to talk to the REAL Nav2 (Now using NavigateToPose!)
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "Interceptor running! Disguised as Nav2. Using Sequential Waypoints.");
    }

private:
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    
    std::vector<geometry_msgs::msg::PoseStamped> pose_queue_;
    bool is_navigating_ = false;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
    {
        if (is_navigating_) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Intercepted a frontier goal from explore_lite!");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
    {
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->succeed(result);

        auto target_pose = goal_handle->get_goal()->pose;
        execute_boustrophedon(target_pose);
    }

    void execute_boustrophedon(const geometry_msgs::msg::PoseStamped& msg)
    {
        std::vector<geometry_msgs::msg::PoseStamped> coverage_poses;
        
        double target_x = msg.pose.position.x;
        double target_y = msg.pose.position.y;
        double sweep_width = 0.35; 
        double zone_size = 1.5;    
        double half_zone = zone_size / 2.0;

        for (double dy = -half_zone; dy <= half_zone; dy += sweep_width) {
            geometry_msgs::msg::PoseStamped pose1;
            geometry_msgs::msg::PoseStamped pose2;
            
            pose1.header.frame_id = "map";
            pose1.header.stamp = this->now();
            pose2.header.frame_id = "map";
            pose2.header.stamp = this->now();
            
            pose1.pose.position.x = target_x - half_zone;
            pose1.pose.position.y = target_y + dy;
            pose1.pose.orientation.w = 1.0;
            
            pose2.pose.position.x = target_x + half_zone;
            pose2.pose.position.y = target_y + dy;
            pose2.pose.orientation.w = 1.0;
            
            int sweep_row = static_cast<int>((dy + half_zone) / sweep_width);
            if (sweep_row % 2 == 0) {
                coverage_poses.push_back(pose1);
                coverage_poses.push_back(pose2);
            } else {
                coverage_poses.push_back(pose2);
                coverage_poses.push_back(pose1);
            }
        }
        
        geometry_msgs::msg::PoseStamped final_pose = msg;
        final_pose.header.frame_id = "map";
        final_pose.header.stamp = this->now();
        final_pose.pose.orientation.w = 1.0; 
        coverage_poses.push_back(final_pose);

        // Load the queue and fire the first waypoint
        pose_queue_ = coverage_poses;
        send_next_pose();
    }

    void send_next_pose()
    {
        if (pose_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Finished Boustrophedon sweep! Ready for next frontier.");
            is_navigating_ = false;
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose_queue_.front();

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                // If successful, pop the completed pose off the queue
                if (!pose_queue_.empty()) {
                    pose_queue_.erase(pose_queue_.begin());
                }
                // Recursively call the function to send the next pose
                send_next_pose(); 
            } else {
                RCLCPP_ERROR(this->get_logger(), "Nav2 failed to reach a sweep waypoint. Aborting sweep.");
                pose_queue_.clear();
                is_navigating_ = false;
            }
        };

        is_navigating_ = true;
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierInterceptor>());
    rclcpp::shutdown();
    return 0;
}