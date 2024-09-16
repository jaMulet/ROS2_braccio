from typing import Dict, Optional, Union

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


class GazeboMixin:
    # === METHODS FOR GAZEBO CLASSIC ===
    @staticmethod
    def include_gazebo_classic(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ),
            **kwargs
        )
    
    @staticmethod
    def node_spawn_entity_classic(
        robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    ) -> Node:
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name")
        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                LaunchConfiguration("robot_name"),
            ],
            output="screen",
            **kwargs
        )
    
    # === METHODS FOR IGNITION GAZEBO ===
    # @staticmethod
    # def include_gazebo(**kwargs) -> IncludeLaunchDescription:
    #     return IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("ros_gz_sim"),
    #                     "launch",
    #                     "gz_sim.launch.py",
    #                 ]
    #             )
    #         ),
    #         **kwargs
    #     )

    # @staticmethod
    # def arg_gz() -> DeclareLaunchArgument:
    #     return DeclareLaunchArgument(
    #         name="gz_args",
    #         default_value="['-r -s -v4 ']",
    #         description="'-s' runs Gazebo server without the GUI client; '-r' starts running simulation immediately; '-v4 ' verbosity level of Gazebo's console output",
    #     )
    
    # @staticmethod
    # def arg_gz_on_exit() -> DeclareLaunchArgument:
    #     return DeclareLaunchArgument(
    #         name="on_exit_shutdown",
    #         default_value="true",
    #         description="Ensures the rest of nodes in the launch file are shutdown",
    #     )

    # @staticmethod
    # def arg_gz_model_var() -> AppendEnvironmentVariable:
    #     return AppendEnvironmentVariable(
    #         'GZ_SIM_RESOURCE_PATH',
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare('braccio_description'),
    #                     'models'
    #             ]
    #         )
    #     )
    
    # === ON GOING === 
    # @staticmethod
    # def node_spawn_entity(
    #     robot_name: Optional[Union[LaunchConfiguration, str]] = None, **kwargs
    # ) -> Node:
    #     if robot_name is None:
    #         robot_name = LaunchConfiguration("robot_name")
    #     return Node(
    #         package="rz_gz_sim",
    #         executable="create",
    #         arguments=[
    #             "-name", braccio,
    #             "-file", urdf_path,
    #         ],
    #         output="screen",
    #         **kwargs
    #     )
        
class BraccioDescriptionMixin:
    @staticmethod
    def param_robot_description(
        model: Optional[Union[LaunchConfiguration, str]] = None,
        robot_name: Optional[Union[LaunchConfiguration, str]] = None,
        sim: Optional[Union[LaunchConfiguration, bool]] = None,
        hw_test: Optional[Union[LaunchConfiguration, bool]] = None,
        base_frame: Optional[Union[LaunchConfiguration, str]] = None,
    ) -> Dict[str, str]:
        if model is None:
            model = LaunchConfiguration("model", default="braccio")
        if robot_name is None:
            robot_name = LaunchConfiguration("robot_name", default="braccio")
        if sim is None:
            sim = LaunchConfiguration("sim", default="true")
        if type(sim) is bool:
            sim = "true" if sim else "false"
        if hw_test is None:
            hw_test = LaunchConfiguration("hw_test", default="false")
        if type(hw_test) is bool:
            hw_test = "true" if hw_test else "false"
        if base_frame is None:
            base_frame = LaunchConfiguration("base_frame", default="world")
        robot_description = {
            "robot_description": Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("braccio_description"),
                            "urdf",
                            model,
                        ]
                    ),
                    ".urdf.xacro",
                    " robot_name:=",
                    robot_name,
                    " sim:=",
                    sim,
                    " base_frame:=",
                    base_frame,
                    " hw_test:=",
                    hw_test,
                ]
            )
        }
        return robot_description

    @staticmethod
    def arg_model() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model",
            default_value="braccio",
            description="The Tinker Braccio model.",
            choices=["braccio"],
        )

    @staticmethod
    def arg_base_frame() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="base_frame",
            default_value="world",
            description="The robot's base frame.",
        )

    @staticmethod
    def arg_robot_name() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="robot_name",
            default_value="braccio",
            description="The robot's name.",
        )

    @staticmethod
    def arg_sim() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sim",
            default_value="true",
            description="Whether to use the simulation or not.",
        )
    
    @staticmethod
    def arg_hw_test() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="hw_test",
            default_value="false",
            description="HW test mode. It enables HWInterface verbosity.",
        )
    
    @staticmethod
    def param_base_frame() -> Dict[str, LaunchConfiguration]:
        return {"base_frame": LaunchConfiguration("base_frame", default="world")}

    @staticmethod
    def param_robot_name() -> Dict[str, LaunchConfiguration]:
        return {"robot_name": LaunchConfiguration("robot_name", default="braccio")}

    @staticmethod
    def param_sim() -> Dict[str, LaunchConfiguration]:
        return {"sim": LaunchConfiguration("sim", default="true")}

    @staticmethod
    def param_hw_test() -> Dict[str, LaunchConfiguration]:
        return {"hw_test": LaunchConfiguration("hw_test", default="false")}

class RVizMixin:
    @staticmethod
    def arg_rviz_config_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config_pkg",
            default_value="braccio_description",
            description="The RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_config() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config",
            default_value="config/config.rviz",
            description="The RViz configuration file.",
        )

    @staticmethod
    def node_rviz(
        rviz_config_pkg: Optional[Union[LaunchConfiguration, str]] = None,
        rviz_config: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs
    ) -> Node:
        if rviz_config_pkg is None:
            rviz_config_pkg = LaunchConfiguration(
                "rviz_config_pkg", default="braccio_description"
            )
        if rviz_config is None:
            rviz_config = LaunchConfiguration(
                "rviz_config", default="config/config.rviz"
            )
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare(rviz_config_pkg),
                        rviz_config,
                    ]
                ),
            ],
            **kwargs
        )