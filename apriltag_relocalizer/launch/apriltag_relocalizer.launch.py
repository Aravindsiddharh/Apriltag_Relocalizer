# apriltag_relocalizer/launch/apriltag_relocalizer.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('apriltag_relocalizer')
    config_path = os.path.join(pkg_dir, 'config', 'apriltag_relocalizer.yaml')
    apriltag_cfg = os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11.yaml')

    return LaunchDescription([

        # === 1. Base → Camera ===
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_camera',
        #     arguments=['0.1', '0.0', '0.3', '0', '0', '0', '1', 'base_link', 'camera_link_optical'],
        #     output='screen'
        # ),

        # === 2. AprilTag Detector WITH TF ===
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_detector',
            output='screen',
            parameters=[
                apriltag_cfg,
                # {
                #     'publish_tf': True,                    # CRITICAL
                #     'tf_frame_prefix': '',                 # DO NOT use 'tag36h11_' → breaks frame name
                #     'camera_frame': 'camera_link_optical',
                #     'add_tag_frames': True,                # REQUIRED
                # }
            ],
            remappings=[
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
                ('detections', '/detections'),
            ],
            arguments=[
                '--ros-args',
                '-r', 'image_rect:=/camera/image_raw',
                '-r', 'camera_info:=/camera/camera_info',
                '--params-file', apriltag_cfg
            ]
        ),

        # === 3. Relocalizer ===
        Node(
            package='apriltag_relocalizer',
            executable='apriltag_relocalizer_node',
            name='apriltag_relocalizer',
            output='screen',
            parameters=[config_path]
        ),
    ])