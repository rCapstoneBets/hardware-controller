import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction

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

        Node(
            package="hardware_controller",
            executable="hardware_node",
            name="hardware_node",
            respawn=True,
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            parameters=[
                LaunchConfiguration("config_file")
            ]
        ),

        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     output='screen',
        #     respawn=True,
        #     arguments=['udp4', '--port', '8888', '-v3'] ## change to -v4 for actual logs
        # ),

        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            output='screen',
            respawn=True,
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ),        
    ])