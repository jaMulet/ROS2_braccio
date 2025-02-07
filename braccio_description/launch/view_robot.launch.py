from launch import LaunchDescription
from launch_ros.actions import Node

from braccio_description import BraccioDescriptionMixin, RVizMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(BraccioDescriptionMixin.arg_model())
    ld.add_action(BraccioDescriptionMixin.arg_robot_name())
    robot_description = BraccioDescriptionMixin.param_robot_description(sim=True)
    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )
    )
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )
    )
    ld.add_action(
        RVizMixin.node_rviz(
            rviz_config_pkg="braccio_description",
            rviz_config="config/config.rviz",
        )
    )
    return ld
