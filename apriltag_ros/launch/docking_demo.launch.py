from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'apriltag_ros'
    apriltag_params = os.path.join(get_package_share_directory(package_name),'cfg','tags_36h11.yaml')

    detection_node = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[('image_rect', 'camera/image_raw'), ('camera_info', 'camera/camera_info'),],
            parameters=[apriltag_params]
        )
    pose_publisher_node = Node(
            package='apriltag_docking_bridge',
            executable='detected_dock_pose_publisher',
            name='detected_dock_pose_publisher',
            parameters=[{ 'parent_frame': 'camera_link_optical', 'child_frame': 'tag36h11_0'}]
        )

    return LaunchDescription([

        detection_node,
        pose_publisher_node,

     ])