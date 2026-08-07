#pragma once

#include "topographic_planner/pathfinder.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace topographic_planner {

/* Action server node for topographic path planning */
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
    /* Executes the path search */
    void execute();

    /* Callback for grid_map subscription */
    void map_callback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

    /* Transforms a pose to a target frame */
    bool transform_pose_to_frame(
        const geometry_msgs::msg::PoseStamped &input_pose,
        const std::string &target_frame,
        const std::string &pose_name,
        geometry_msgs::msg::PoseStamped &output_pose
    );

    /* Publishes search tree visualization markers */
    void publish_search_tree(
        const grid_map::Position &p1,
        const grid_map::Position &p2,
        int type
    );

    // Node state
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Pub/subs and action server
    std::unique_ptr<nav2_util::SimpleActionServer<ComputePathToPose>>
        action_server_;
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

    // Pathfinder object
    pathfinder::Pathfinder pathfinder_;
};

}    // namespace topographic_planner
