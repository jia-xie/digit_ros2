from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    digit_communication_node = Node(
        package="digit_hardware",
        executable="DigitCommunicationNode",
        name="digit_communication_node",
        parameters=[{'run_simulation': True}]
    )

    ld.add_action(digit_communication_node)

    return ld
