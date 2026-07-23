#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace topographic_planner {

class TopographicPlannerActionServer : public nav2_util::LifecycleNode {
  public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePathToPose =
        rclcpp_action::ServerGoalHandle<ComputePathToPose>;

    explicit TopographicPlannerActionServer(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );
    ~TopographicPlannerActionServer() override;

  protected:
    nav2_util::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &state) override;

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
    void
    execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle);

    void map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

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

    // Parameters
    float resolution_;
    int smooth_steps_;
    float weight_grad_;
    float weight_xy_dist_;
    float weight_cost_;
    float weight_heuristic_;
    float max_gradient_;
    std::string elevation_layer_;
    std::string traversability_layer_;
};

} // namespace topographic_planner
