import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sabry_pkg = get_package_share_directory("sabry")
    
    # Declare model argument (URDF/XACRO)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([
            sabry_pkg, "urdf", "sabry.urdf.xacro"
        ]),
        description="Absolute path to robot URDF/XACRO file"
    )

    # Set GZ_SIM_RESOURCE_PATH to find models/worlds
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(sabry_pkg).parent.resolve())  # Points to workspace src/ directory
    )

    # Robot description from XACRO
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=true"
        ]),
        value_type=str
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    # CORRECT WAY: Resolve world path as a single string argument
    simulation_world_file_path = Path(sabry_pkg, "worlds/tool_changer.sdf").as_posix()

    # Launch Gazebo with custom world - FIXED ARGUMENT FORMATTING
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ]),
        launch_arguments={'gz_args': '-v 4 -r ' + simulation_world_file_path}.items()
    )

    # Spawn robot entity
    # gz_spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     output="screen",
    #     # arguments=[
    #     #     "-topic", "robot_description",
    #     #     "-name", "sabry",
    #     #     # "-x", "0.0",
    #     #     # "-y", "0.0",
    #     #     # "-z", "0.6",  # Lift above ground plane
    #     #     # "-R", "0.0",
    #     #     # "-P", "0.0",
    #     #     # "-Y", "0.0"
    #     # ]
    # )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "sabry"],
    )


    # Clock bridge (essential for sim_time)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        gz_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])