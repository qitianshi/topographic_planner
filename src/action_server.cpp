#include "topographic_planner/action_server.hpp"
#include "topographic_planner/pathfinder.hpp"
#include "topographic_planner/smooth.hpp"

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace topographic_planner {

TopographicPlannerActionServer::TopographicPlannerActionServer(
    const rclcpp::NodeOptions &options
)
    : nav2_util::LifecycleNode("topographic_planner_node", "", options) {}

TopographicPlannerActionServer::~TopographicPlannerActionServer() = default;

nav2_util::CallbackReturn TopographicPlannerActionServer::on_configure(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO(get_logger(), "Configuring topographic_planner_node...");

    nav2_util::declare_parameter_if_not_declared(
        this, "resolution", rclcpp::ParameterValue(0.1)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "smooth_steps", rclcpp::ParameterValue(10)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "weight_grad", rclcpp::ParameterValue(1.26)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "weight_xy_dist", rclcpp::ParameterValue(1.34)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "weight_cost", rclcpp::ParameterValue(0.00706)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "weight_heuristic", rclcpp::ParameterValue(3.5)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "max_gradient", rclcpp::ParameterValue(0.577)
    );
    nav2_util::declare_parameter_if_not_declared(
        this, "elevation_layer_name", rclcpp::ParameterValue("elevation")
    );
    nav2_util::declare_parameter_if_not_declared(
        this,
        "traversability_layer_name",
        rclcpp::ParameterValue("traversability")
    );

    get_parameter("resolution", resolution_);
    get_parameter("smooth_steps", smooth_steps_);
    get_parameter("weight_grad", weight_grad_);
    get_parameter("weight_xy_dist", weight_xy_dist_);
    get_parameter("weight_cost", weight_cost_);
    get_parameter("weight_heuristic", weight_heuristic_);
    get_parameter("max_gradient", max_gradient_);
    get_parameter("elevation_layer_name", elevation_layer_);
    get_parameter("traversability_layer_name", traversability_layer_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "grid_map",
        rclcpp::QoS(1).transient_local(),
        std::bind(&TopographicPlannerActionServer::map_callback, this, _1)
    );

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "search_tree", 10
    );

    action_server_ = rclcpp_action::create_server<ComputePathToPose>(
        this,
        "compute_path_to_pose",
        std::bind(&TopographicPlannerActionServer::handle_goal, this, _1, _2),
        std::bind(&TopographicPlannerActionServer::handle_cancel, this, _1),
        std::bind(&TopographicPlannerActionServer::handle_accepted, this, _1)
    );

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerActionServer::on_activate(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO(get_logger(), "Activating topographic_planner_node...");
    marker_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerActionServer::on_deactivate(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO(get_logger(), "Deactivating topographic_planner_node...");
    marker_pub_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerActionServer::on_cleanup(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO(get_logger(), "Cleaning up topographic_planner_node...");
    action_server_.reset();
    map_sub_.reset();
    marker_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn TopographicPlannerActionServer::on_shutdown(
    const rclcpp_lifecycle::State & /*state*/
) {
    RCLCPP_INFO(get_logger(), "Shutting down topographic_planner_node...");
    return nav2_util::CallbackReturn::SUCCESS;
}

void TopographicPlannerActionServer::map_callback(
    const grid_map_msgs::msg::GridMap::SharedPtr msg
) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!cached_map_) {
        cached_map_ = std::make_shared<grid_map::GridMap>();
    }
    grid_map::GridMapRosConverter::fromMessage(*msg, *cached_map_);
    map_frame_id_ = msg->header.frame_id;
}

rclcpp_action::GoalResponse TopographicPlannerActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ComputePathToPose::Goal> /*goal*/
) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TopographicPlannerActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> /*goal_handle*/
) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TopographicPlannerActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle
) {
    std::thread{
        std::bind(&TopographicPlannerActionServer::execute, this, _1),
        goal_handle
    }
        .detach();
}

void TopographicPlannerActionServer::execute(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle
) {
    auto result = std::make_shared<ComputePathToPose::Result>();
    auto goal = goal_handle->get_goal();

    std::shared_ptr<grid_map::GridMap> map_copy;
    std::string map_frame;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!cached_map_) {
            RCLCPP_ERROR(get_logger(), "No grid map received yet!");
            goal_handle->abort(result);
            return;
        }
        map_copy = std::make_shared<grid_map::GridMap>(*cached_map_);
        map_frame = map_frame_id_;
    }

    geometry_msgs::msg::PoseStamped start_pose, goal_pose;

    // transform start
    try {
        if (!tf_buffer_->canTransform(
                map_frame,
                goal->start.header.frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)
            )) {
            RCLCPP_ERROR(
                get_logger(), "Could not transform start to map frame"
            );
            result->error_code = ComputePathToPose::Result::TF_ERROR;
            goal_handle->abort(result);
            return;
        }
        tf_buffer_->transform(goal->start, start_pose, map_frame);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF exception: %s", ex.what());
        result->error_code = ComputePathToPose::Result::TF_ERROR;
        goal_handle->abort(result);
        return;
    }

    // transform goal
    try {
        if (!tf_buffer_->canTransform(
                map_frame,
                goal->goal.header.frame_id,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)
            )) {
            RCLCPP_ERROR(get_logger(), "Could not transform goal to map frame");
            result->error_code = ComputePathToPose::Result::TF_ERROR;
            goal_handle->abort(result);
            return;
        }
        tf_buffer_->transform(goal->goal, goal_pose, map_frame);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF exception: %s", ex.what());
        result->error_code = ComputePathToPose::Result::TF_ERROR;
        goal_handle->abort(result);
        return;
    }

    pathfinder::Config config;
    config.weights.wxyDist = weight_xy_dist_;
    config.weights.wgrad = weight_grad_;
    config.weights.wcost = weight_cost_;
    config.weights.wheuristic = weight_heuristic_;
    config.weights.maxGrad = max_gradient_;
    config.elevation_layer = elevation_layer_;
    config.traversability_layer = traversability_layer_;

    config.start.x() = start_pose.pose.position.x;
    config.start.y() = start_pose.pose.position.y;
    config.dest.x() = goal_pose.pose.position.x;
    config.dest.y() = goal_pose.pose.position.y;

    if (marker_pub_->get_subscription_count() > 0) {
        config.visualization_callback = [this, map_frame](
                                            const grid_map::Position &p1,
                                            const grid_map::Position &p2,
                                            int type
                                        ) {
            (void)type; // just lines for now
            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame;
            marker.header.stamp = now();
            marker.ns = "search_tree";
            marker.id = 0; // or increment
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
        };
    }

    std::vector<grid_map::Position> path;
    pathfinder::Result pf_result = pathfinder::search(*map_copy, config, path);

    if (pf_result != pathfinder::Result::FoundPath) {
        RCLCPP_ERROR(
            get_logger(),
            "Pathfinder failed: %s",
            pathfinder::resultToStr(pf_result)
        );
        if (pf_result == pathfinder::Result::StartOutsideMap) {
            result->error_code = ComputePathToPose::Result::START_OUTSIDE_MAP;
        } else if (pf_result == pathfinder::Result::GoalOutsideMap) {
            result->error_code = ComputePathToPose::Result::GOAL_OUTSIDE_MAP;
        } else {
            result->error_code = ComputePathToPose::Result::NO_VALID_PATH;
        }
        goal_handle->abort(result);
        return;
    }

    if (smooth_steps_ > 0) {
        smooth::apply(*map_copy, smooth_steps_, path);
    }

    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = map_frame;
    ros_path.header.stamp = now();
    for (const auto &p : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        ros_path.poses.push_back(pose);
    }

    result->path = ros_path;
    goal_handle->succeed(result);
}

} // namespace topographic_planner
