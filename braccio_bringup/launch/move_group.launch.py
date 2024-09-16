from typing import List

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from braccio_bringup import BraccioMoveGroupMixin
from braccio_description import BraccioDescriptionMixin, RVizMixin


def launch_setup(context) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    ld.add_action(BraccioMoveGroupMixin.arg_allow_trajectory_execution())
    ld.add_action(BraccioMoveGroupMixin.arg_capabilities())
    ld.add_action(BraccioMoveGroupMixin.arg_disable_capabilities())
    ld.add_action(BraccioMoveGroupMixin.arg_monitor_dynamics())
    ld.add_action(BraccioMoveGroupMixin.args_publish_monitored_planning_scene())

    model = LaunchConfiguration("model").perform(context)
    moveit_configs_builder = BraccioMoveGroupMixin.moveit_configs_builder(
        model,
        package_name=f"{model}_moveit_config",
    )
    movegroup_params = BraccioMoveGroupMixin.params_move_group()

    # MoveGroup
    ld.add_action(
        BraccioMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                movegroup_params,
                {"use_sim_time": LaunchConfiguration("sim")},
            ],
        )
    )

    # RViz
    rviz = RVizMixin.node_rviz(
        rviz_config_pkg=f"{model}_moveit_config",
        rviz_config="config/moveit.rviz",
        parameters=BraccioMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        ),
    )

    ld.add_action(rviz)

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(BraccioDescriptionMixin.arg_model())
    ld.add_action(BraccioDescriptionMixin.arg_robot_name())
    ld.add_action(BraccioDescriptionMixin.arg_sim())

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
