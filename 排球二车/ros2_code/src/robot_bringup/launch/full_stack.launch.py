from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    base_controller_share = get_package_share_directory("base_controller")
    hit_controller_share = get_package_share_directory("hit_controller")
    pitch_controller_share = get_package_share_directory("pitch_controller")
    dm_driver_share = get_package_share_directory("vobot_dm_driver")
    teleop_share = get_package_share_directory("teleop_twist_joy")

    joy_config = f"{bringup_share}/config/joy.yaml"
    base_controller_config = f"{base_controller_share}/config/base_controller.yaml"
    joy_hit_trigger_config = f"{hit_controller_share}/config/joy_hit_trigger.yaml"
    pitch_controller_config = f"{pitch_controller_share}/config/pitch_controller.yaml"
    dm_driver_config = f"{dm_driver_share}/config/vobot_dm_driver.yaml"
    dm_hit_motor_config = f"{dm_driver_share}/config/dm_hit_motor.yaml"
    dm_pitch_motor_config = f"{dm_driver_share}/config/dm_pitch_motor.yaml"
    teleop_launch = f"{teleop_share}/launch/teleop-launch.py"

    return LaunchDescription([
        Node(
            package="vobot_dm_driver",
            executable="dm_motor_node",
            name="dm_motor_node",
            output="screen",
            parameters=[dm_driver_config],
        ),
        Node(
            package="base_controller",
            executable="base_controller_node",
            name="base_controller_node",
            output="screen",
            parameters=[base_controller_config],
        ),
        Node(
            package="pitch_controller",
            executable="pitch_controller_node",
            name="pitch_controller_node",
            output="screen",
            parameters=[pitch_controller_config],
        ),
        Node(
            package="hit_controller",
            executable="joy_hit_trigger_node",
            name="joy_hit_trigger_node",
            output="screen",
            parameters=[joy_hit_trigger_config],
        ),
        Node(
            package="vobot_dm_driver",
            executable="dm_pitch_motor_node",
            name="dm_pitch_motor_node",
            output="screen",
            parameters=[dm_pitch_motor_config],
        ),
        Node(
            package="vobot_dm_driver",
            executable="dm_hit_motor_node",
            name="dm_hit_motor_node",
            output="screen",
            parameters=[dm_hit_motor_config],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch),
            launch_arguments={"config_filepath": joy_config}.items(),
        ),
    ])
