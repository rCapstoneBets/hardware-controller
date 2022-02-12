import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    model_path = PathJoinSubstitution([
        get_package_share_directory('hardware_controller'),
        'cfg',
        'hardware_node.yaml'
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "config_file", 
            default_value=model_path,
            description="path to the config to use",
        ),

        DeclareLaunchArgument(
            "log_level", 
            default_value="INFO",
            description="log level to use",
        ),

        launch_ros.actions.Node(
            package="hardware_controller",
            executable="hardware_node",
            name="hardware_node",
            output="screen",
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            parameters=[
                LaunchConfiguration("config_file")
            ]
        ),
    ])