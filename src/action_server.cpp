#include "topographic_planner/action_server.hpp"
#include "topographic_planner/pathfinder.hpp"
#include "topographic_planner/smooth.hpp"

#include <cmath>
#include <nav2_util/geometry_utils.hpp>
#include <nav2_util/node_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

constexpr const char *node_name = "topographic_planner_node";
constexpr const char *action_name = "compute_path_to_pose";
constexpr const char *grid_map_topic = "grid_map";
constexpr const char *search_tree_topic = "search_tree";

// ROS param names
constexpr const char *param_resolution = "resolution";
constexpr const char *param_smooth_steps = "smooth_steps";
constexpr const char *param_weight_grad = "weight_grad";
constexpr const char *param_weight_xy_dist = "weight_xy_dist";
constexpr const char *param_weight_cost = "weight_cost";
constexpr const char *param_weight_heuristic = "weight_heuristic";
constexpr const char *param_max_gradient = "max_gradient";
constexpr const char *param_elev_layer_name = "elevation_layer_name";
constexpr const char *param_trav_layer_name = "traversability_layer_name";

// ROS param default values
constexpr double default_resolution = 0.1;
constexpr int default_smooth_steps = 10;
constexpr double default_weight_grad = 1.26;
constexpr double default_weight_xy_dist = 1.34;
constexpr double default_weight_cost = 0.00706;
constexpr double default_weight_heuristic = 3.5;
constexpr double default_max_gradient = 0.577;
constexpr const char *default_elev_layer_name = "elevation";
constexpr const char *default_trav_layer_name = "traversability";

constexpr double tf_timeout_sec = 1.0;

/** Converts `pathfinder::Result` to `ComputePathToPose::Result` */
uint16_t to_action_result(pathfinder::Result result) {
    switch (result) {
    case pathfinder::Result::FoundPath:
        return nav2_msgs::action::ComputePathToPose::Result::NONE;
    case pathfinder::Result::NoPath:
        return nav2_msgs::action::ComputePathToPose::Result::NO_VALID_PATH;
    case pathfinder::Result::StartOutsideMap:
        return nav2_msgs::action::ComputePathToPose::Result::START_OUTSIDE_MAP;
    case pathfinder::Result::StartOccupied:
        return nav2_msgs::action::ComputePathToPose::Result::START_OCCUPIED;
    case pathfinder::Result::GoalOutsideMap:
        return nav2_msgs::action::ComputePathToPose::Result::GOAL_OUTSIDE_MAP;
    case pathfinder::Result::GoalOccupied:
        return nav2_msgs::action::ComputePathToPose::Result::GOAL_OCCUPIED;
    case pathfinder::Result::Cancelled:
        return nav2_msgs::action::ComputePathToPose::Result::NONE;
    default:
        return nav2_msgs::action::ComputePathToPose::Result::UNKNOWN;
    }
}

/** Converts a `pathfinder::Path` to `nav_msgs::msg::Path` */
nav_msgs::msg::Path to_ros_path(
    const pathfinder::Path &pf_path,
    const std::string &frame_id,
    const rclcpp::Time &stamp,
    const geometry_msgs::msg::Quaternion &goal_orientation
) {
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = frame_id;
    ros_path.header.stamp = stamp;

    for (size_t i = 0; i < pf_path.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x = pf_path[i].x();
        pose.pose.position.y = pf_path[i].y();
        pose.pose.position.z = 0.0;

        if (i < pf_path.size() - 1) {
            double yaw = std::atan2(
                pf_path[i + 1].y() - pf_path[i].y(),
                pf_path[i + 1].x() - pf_path[i].x()
            );
            pose.pose.orientation =
                nav2_util::geometry_utils::orientationAroundZAxis(yaw);
        } else {
            pose.pose.orientation = goal_orientation;
        }

        ros_path.poses.push_back(pose);
    }
    return ros_path;
}

}    // namespace

namespace topographic_planner {

TopographicPlannerNode::TopographicPlannerNode(
    const rclcpp::NodeOptions &options
)
    : nav2_util::LifecycleNode(node_name, "", options) {}

TopographicPlannerNode::~TopographicPlannerNode() = default;

nav2_util::CallbackReturn TopographicPlannerNode::on_configure(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO_STREAM(get_logger(), "Configuring " << node_name << "...");

    auto declare_param = [this](const std::string &name, auto default_value) {
        nav2_util::declare_parameter_if_not_declared(
            this, name, rclcpp::ParameterValue(default_value)
        );
    };

    declare_param(param_resolution, default_resolution);
    declare_param(param_smooth_steps, default_smooth_steps);
    declare_param(param_weight_grad, default_weight_grad);
    declare_param(param_weight_xy_dist, default_weight_xy_dist);
    declare_param(param_weight_cost, default_weight_cost);
    declare_param(param_weight_heuristic, default_weight_heuristic);
    declare_param(param_max_gradient, default_max_gradient);
    declare_param(param_elev_layer_name, default_elev_layer_name);
    declare_param(param_trav_layer_name, default_trav_layer_name);

    // Retrieves params
    get_parameter(param_resolution, resolution_);
    get_parameter(param_smooth_steps, smooth_steps_);
    get_parameter(param_weight_grad, config_.weights.grad);
    get_parameter(param_weight_xy_dist, config_.weights.xy_dist);
    get_parameter(param_weight_cost, config_.weights.cost);
    get_parameter(param_weight_heuristic, config_.weights.heuristic);
    get_parameter(param_max_gradient, config_.weights.max_grad);
    get_parameter(param_elev_layer_name, config_.elev_layer);
    get_parameter(param_trav_layer_name, config_.trav_layer);

    // tf2 buffer and listener for transforming start/goal poses.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Map subscription
    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
        grid_map_topic,
        rclcpp::QoS(1).transient_local(),
        std::bind(
            &TopographicPlannerNode::map_callback, this, std::placeholders::_1
        )
    );

    // Search tree debug publisher
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        search_tree_topic, 10
    );

    // Callback for publishing a new search tree edge for debugging.
    config_.visualization_callback = std::bind(
        &TopographicPlannerNode::publish_search_tree,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
    );

    // Compute path action
    action_server_ =
        std::make_unique<nav2_util::SimpleActionServer<ComputePathToPose>>(
            this, action_name, std::bind(&TopographicPlannerNode::execute, this)
        );

    // Callback for checking cancellation
    config_.abort_check_callback = [this]() {
        return action_server_->is_cancel_requested() ||
               action_server_->is_preempt_requested();
    };

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerNode::on_activate(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO_STREAM(get_logger(), "Activating " << node_name << "...");
    marker_pub_->on_activate();
    action_server_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerNode::on_deactivate(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO_STREAM(get_logger(), "Deactivating " << node_name << "...");
    marker_pub_->on_deactivate();
    action_server_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerNode::on_cleanup(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO_STREAM(get_logger(), "Cleaning up " << node_name << "...");
    action_server_.reset();
    map_sub_.reset();
    marker_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    std::lock_guard<std::mutex> lock(map_mutex_);
    cached_map_.reset();

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerNode::on_shutdown(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO_STREAM(get_logger(), "Shutting down " << node_name << "...");
    return nav2_util::CallbackReturn::SUCCESS;
}

void TopographicPlannerNode::map_callback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg
) {
    // Updates the map cache with a mutex.
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!cached_map_) {
        cached_map_ = std::make_shared<grid_map::GridMap>();
    }
    grid_map::GridMapRosConverter::fromMessage(*msg, *cached_map_);
    map_frame_id_ = msg->header.frame_id;
}

void TopographicPlannerNode::publish_search_tree(
    const grid_map::Position &p1,
    const grid_map::Position &p2,
    int type
) {
    (void)type;    // Just lines for now
    if (!marker_pub_ || marker_pub_->get_subscription_count() == 0) {
        return;
    }
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = now();
    marker.ns = search_tree_topic;
    marker.id = 0;    // TODO: Increment?
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    geometry_msgs::msg::Point pt1, pt2;
    pt1.x = p1.x();
    pt1.y = p1.y();
    pt1.z = 0.0;
    pt2.x = p2.x();
    pt2.y = p2.y();
    pt2.z = 0.0;
    marker.points.push_back(pt1);
    marker.points.push_back(pt2);
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

bool TopographicPlannerNode::transform_pose_to_frame(
    const geometry_msgs::msg::PoseStamped &input_pose,
    const std::string &target_frame,
    const std::string &pose_name,
    geometry_msgs::msg::PoseStamped &output_pose
) {
    try {

        // Checks if tf is available.
        if (!tf_buffer_->canTransform(
                target_frame,
                input_pose.header.frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(tf_timeout_sec)
            )) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Could not transform " << pose_name << " to map frame '"
                                       << target_frame << "' from frame '"
                                       << input_pose.header.frame_id << "'"
            );
            return false;
        }

        // Performs the transformation.
        tf_buffer_->transform(input_pose, output_pose, target_frame);
        return true;

    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "TF exception transforming " << pose_name << ": " << ex.what()
        );
        return false;
    }
}

void TopographicPlannerNode::execute() {
    auto result = std::make_shared<ComputePathToPose::Result>();
    auto goal = action_server_->get_current_goal();

    // Makes a stable copy of the most recent cached map.
    std::shared_ptr<grid_map::GridMap> map_copy;
    std::string map_frame;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!cached_map_) {
            RCLCPP_ERROR_STREAM(get_logger(), "No grid map received yet!");
            action_server_->terminate_current(result);
            return;
        }
        map_copy = std::make_shared<grid_map::GridMap>(*cached_map_);
        map_frame = map_frame_id_;
    }

    // Tries to transform poses to the map frame, or aborts if failed.
    geometry_msgs::msg::PoseStamped start_pose, goal_pose;
    if (!transform_pose_to_frame(goal->start, map_frame, "start", start_pose) ||
        !transform_pose_to_frame(goal->goal, map_frame, "goal", goal_pose)) {
        result->error_code = ComputePathToPose::Result::TF_ERROR;
        action_server_->terminate_current(result);
        return;
    }

    // Converts ROS pose to Position for easier maths.
    grid_map::Position start_pos(
        start_pose.pose.position.x, start_pose.pose.position.y
    );
    grid_map::Position dest_pos(
        goal_pose.pose.position.x, goal_pose.pose.position.y
    );

    // Performs search.
    pathfinder::Path path;
    pathfinder::Result pf_result =
        pathfinder_.search(*map_copy, config_, start_pos, dest_pos, path);

    if (pf_result == pathfinder::Result::Cancelled) {
        RCLCPP_INFO_STREAM(get_logger(), "Pathfinder search cancelled.");
        action_server_->terminate_current(result);
        return;
    }

    if (pf_result != pathfinder::Result::FoundPath) {
        result->error_code = to_action_result(pf_result);
        action_server_->terminate_current(result);
        return;
    }

    // Smooths the path if requested.
    if (smooth_steps_ > 0) {
        smooth::apply(*map_copy, config_.trav_layer, smooth_steps_, path);
    }

    result->path =
        to_ros_path(path, map_frame, now(), goal_pose.pose.orientation);
    action_server_->succeeded_current(result);
}

}    // namespace topographic_planner
