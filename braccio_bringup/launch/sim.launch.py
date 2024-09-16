from typing import List

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution

from braccio_bringup import BraccioMoveGroupMixin
from braccio_description import BraccioDescriptionMixin, RVizMixin, GazeboMixin
from braccio_hardware import BraccioHardwareInterfaceMixin


def launch_setup(context: LaunchContext) -> List[LaunchDescriptionEntity]:
    ld = LaunchDescription()

    robot_description = BraccioDescriptionMixin.param_robot_description(sim=True)
    ld.add_action(GazeboMixin.include_gazebo_classic())  # Gazebo has its own controller manager
    spawn_entity = GazeboMixin.node_spawn_entity_classic()
    ld.add_action(spawn_entity)
    joint_state_broadcaster = BraccioHardwareInterfaceMixin.node_joint_state_broadcaster()
    ld.add_action(joint_state_broadcaster)
    robot_state_publisher = BraccioHardwareInterfaceMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=True, frame_prefix=""
    )
 
    ld.add_action(robot_state_publisher)
    # Do not condition robot state publisher on joint state broadcaster as Gazebo uses robot state publisher to retrieve robot description
    ld.add_action(BraccioHardwareInterfaceMixin.node_arm_controller())
    ld.add_action(BraccioHardwareInterfaceMixin.node_gripper_controller())

    # MoveIt 2
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

    ld.add_action(
        BraccioMoveGroupMixin.node_move_group(
            parameters=[
                moveit_configs_builder.to_dict(),
                movegroup_params,
                {"use_sim_time": True},
            ],
            condition=IfCondition(LaunchConfiguration("moveit")),
        )
    )

    # RViz and MoveIt
    rviz_moveit = RVizMixin.node_rviz(
        rviz_config_pkg=f"{model}_moveit_config",
        rviz_config="config/moveit.rviz",
        parameters=BraccioMoveGroupMixin.params_rviz(
            moveit_configs=moveit_configs_builder.to_moveit_configs()
        ),
        condition=IfCondition(
            AndSubstitution(LaunchConfiguration("moveit"), LaunchConfiguration("rviz"))
        ),
    )

    # RViz no MoveIt
    ld.add_action(RVizMixin.arg_rviz_config_pkg())
    ld.add_action(RVizMixin.arg_rviz_config())
    rviz = RVizMixin.node_rviz(
        condition=IfCondition(
            AndSubstitution(
                LaunchConfiguration("rviz"),
                NotSubstitution(LaunchConfiguration("moveit")),
            )
        )
    )

    # RViz event handler
    rviz_event_handler = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[rviz_moveit, rviz])
    )
    ld.add_action(rviz_event_handler)

    return ld.entities


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(BraccioDescriptionMixin.arg_model())
    ld.add_action(BraccioDescriptionMixin.arg_robot_name())
    ld.add_action(
        DeclareLaunchArgument(
            name="moveit",
            default_value="true",
            description="Whether to launch MoveIt 2.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            name="rviz", default_value="true", description="Whether to launch RViz."
        )
    )
    ld.add_action(BraccioHardwareInterfaceMixin.arg_arm_ctrl())
    ld.add_action(BraccioHardwareInterfaceMixin.arg_gripper_ctrl())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
