import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

def generate_launch_description():
    pkg_dir = get_package_share_directory('topographic_planner')
    params_file = os.path.join(pkg_dir, 'config', 'planner_params.yaml')

    planner_node = LifecycleNode(
        package='topographic_planner',
        executable='topographic_planner_node',
        name='topographic_planner_node',
        namespace='',
        output='screen',
        parameters=[params_file]
    )

    emit_event_to_request_that_node_does_configure_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('topographic_planner_node'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_event_to_request_that_node_does_activate_transition = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_node_name('topographic_planner_node'),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    on_configure_success = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=planner_node,
            goal_state='inactive',
            entities=[
                emit_event_to_request_that_node_does_activate_transition,
            ],
        )
    )

    return LaunchDescription([
        planner_node,
        emit_event_to_request_that_node_does_configure_transition,
        on_configure_success
    ])
