#pragma once

#include "topographic_planner/pathfinder.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace topographic_planner {

class TopographicPlannerNode : public nav2_util::LifecycleNode {
  public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePathToPose =
        rclcpp_action::ServerGoalHandle<ComputePathToPose>;

    explicit TopographicPlannerNode(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );
    ~TopographicPlannerNode() override;

  protected:
    nav2_util::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &state
    ) override;

    nav2_util::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &state
    ) override;

    nav2_util::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &state
    ) override;

    nav2_util::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &state
    ) override;

    nav2_util::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &state
    ) override;

  private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ComputePathToPose::Goal> goal
    );

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleComputePathToPose> goal_handle
    );

    void handle_accepted(
        const std::shared_ptr<GoalHandleComputePathToPose> goal_handle
    );

    void execute(
        const std::shared_ptr<GoalHandleComputePathToPose> goal_handle
    );

    void map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

    /* Transforms a pose to a target frame. */
    bool transform_pose_to_frame(
        const geometry_msgs::msg::PoseStamped &input_pose,
        const std::string &target_frame,
        const std::string &pose_name,
        geometry_msgs::msg::PoseStamped &output_pose
    );

    // Node state
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
    rclcpp_lifecycle::LifecyclePublisher<
        visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // Map cache
    std::mutex map_mutex_;
    std::shared_ptr<grid_map::GridMap> cached_map_;
    std::string map_frame_id_;

    // Configuration & parameters
    pathfinder::Config config_;
    float resolution_;
    int smooth_steps_;
};

} // namespace topographic_planner
