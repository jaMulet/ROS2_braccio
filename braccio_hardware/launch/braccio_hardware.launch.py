from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from braccio_description import BraccioDescriptionMixin
from braccio_hardware import BraccioHardwareInterfaceMixin


class BraccioHardwareInterface(BraccioDescriptionMixin, BraccioHardwareInterfaceMixin):
    pass


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(BraccioHardwareInterface.arg_base_frame())
    ld.add_action(BraccioHardwareInterface.arg_model())
    ld.add_action(BraccioHardwareInterface.arg_robot_name())
    robot_description = BraccioHardwareInterface.param_robot_description(sim=False)
    ld.add_action(BraccioHardwareInterface.arg_ctrl_cfg_pkg())
    ld.add_action(BraccioHardwareInterface.arg_ctrl_cfg())
    ld.add_action(BraccioHardwareInterface.arg_ctrl())
    ld.add_action(BraccioHardwareInterface.arg_frame_prefix())
    ros2_control_node = BraccioHardwareInterface.node_ros2_control(
        robot_description=robot_description
    )
    ld.add_action(ros2_control_node)
    joint_state_broadcaster = BraccioHardwareInterface.node_joint_state_broadcaster()
    controller = BraccioHardwareInterface.node_controller()
    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster, controller],
        )
    )
    ld.add_action(controller_event_handler)
    robot_state_publisher = BraccioHardwareInterface.node_robot_state_publisher(
        robot_description=robot_description,
        use_sim_time=False,
        frame_prefix=BraccioHardwareInterface.param_frame_prefix(),
    )
    ld.add_action(robot_state_publisher)
    return ld
