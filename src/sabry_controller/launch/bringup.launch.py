import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ----------------------------
    # Packages
    # ----------------------------
    sabry_pkg = get_package_share_directory("sabry")
    sabry_moveit_pkg = get_package_share_directory("sabry_moveit")

    # ----------------------------
    # Launch Arguments
    # ----------------------------
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Run in simulation (true) or real hardware (false)"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution([
            sabry_pkg, "urdf", "sabry.urdf.xacro"
        ]),
        description="Absolute path to robot URDF/XACRO"
    )

    is_sim = LaunchConfiguration("is_sim")

    # ----------------------------
    # Robot Description
    # ----------------------------
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=", is_sim
        ]),
        value_type=str
    )

    # ----------------------------
    # Robot State Publisher
    # ----------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": is_sim
        }],
        output="screen"
    )

    # ----------------------------
    # ros2_control (REAL ROBOT ONLY)
    # ----------------------------
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        condition=UnlessCondition(is_sim),
        parameters=[
            {"robot_description": robot_description},
            os.path.join(
                sabry_moveit_pkg,
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(is_sim),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        condition=UnlessCondition(is_sim),
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ----------------------------
    # Gazebo (SIM ONLY)
    # ----------------------------
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(sabry_pkg).parent.resolve())
    )

    world_path = Path(sabry_pkg, "worlds", "tool_changer.sdf").as_posix()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ]),
        condition=IfCondition(is_sim),
        launch_arguments={
            "gz_args": "-v 4 -r " + world_path
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        condition=IfCondition(is_sim),
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "sabry"
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        condition=IfCondition(is_sim),
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    # ----------------------------
    # Launch Description
    # ----------------------------
    return LaunchDescription([
        is_sim_arg,
        model_arg,

        gz_resource_path,

        robot_state_publisher_node,

        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,

        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
