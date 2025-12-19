# AprilTag Relocalizer

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)  
<!-- Add other badges as needed, e.g., build status, version, etc. -->

A robust relocalization module using **AprilTag** fiducial markers for robotics applications. This tool enables accurate pose correction (relocalization) in visual odometry or SLAM systems by detecting known AprilTags in the environment and adjusting the robot/camera pose accordingly. Ideal for reducing drift in GPS-denied environments, warehouse navigation, or competition robotics (e.g., FRC/FTC).

<grok-card data-id="f2e6f5" data-type="image_card"></grok-card>



<grok-card data-id="5d7e18" data-type="image_card"></grok-card>



<grok-card data-id="5edeec" data-type="image_card"></grok-card>


## What are AprilTags?

AprilTags are visual fiducial markers similar to QR codes but optimized for **robust detection at longer ranges**, low overhead, and precise 3D pose estimation. They are widely used in robotics for localization, camera calibration, and augmented reality.

<grok-card data-id="fe19d5" data-type="image_card"></grok-card>



<grok-card data-id="49a3e5" data-type="image_card"></grok-card>



<grok-card data-id="e4e5df" data-type="image_card"></grok-card>


Example of AprilTag detection in a camera feed:

<grok-card data-id="c5f2f0" data-type="image_card"></grok-card>



<grok-card data-id="06a178" data-type="image_card"></grok-card>


## Features

- Real-time AprilTag detection and pose estimation
- Relocalization by fusing tag observations with existing odometry/SLAM estimates
- Support for multiple tag families (e.g., tag36h11, tagStandard41h12)
- Configurable tag map (known positions of fixed tags in the environment)
- Low computational overhead for embedded platforms
- Integration with [AprilTag library](https://github.com/AprilRobotics/apriltag) (version 3 recommendedun)


## Initial Setup

- Make sure that all the packages are installed regarding apriltags
- Unzip or clone the apriltag package given
- Then source the package that is given.
- Make sure that the tag's positions are hardcoded in a yaml file (example : tags.yaml)
## Installation

```bash
sudo apt install ros-humble-apriltag-msgs
cd apriltag_ws
colcon build
source install/setup.bash
```
## Start the Process

1. Place the apriltag in the real or gazebo environment and save it.
2. When the robot is lost in map make the robot to move to nearest apriltag position using teleoperation or simple navigation. So that the robot can detect it and extend its tf_tree to tag frame.

```bash
ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/camera/image -r camera_info:=/camera/camera_info --params-file ~/apriltag_ws/src/apriltag_ros/cfg/tags_36h11.yaml
```
 In order to view the tf tree, run using
```bash
ros2 run tf2_tools view_frames
```

3. Then after placing the robot in front of the apriltag or deflection 
, run 
```bash
ros2 run apriltag_relocalizer apriltag_relocalizer_node 
```
4. The tag format should be like this.

```bash
# config/tags.yaml   ←  DO NOT change the indentation!
tags:
  0:
    x: -8.95423
    y: -4.01118
    yaw: -3.14

  1:
    x: -6.78457
    y: -6.35085
    yaw: -1.57

  2:
    x: -4.01422
    y: -2.59672
    yaw: 1.57
```
5. If the reloalization wants to be a continuous one, launch 
```bash
ros2 launch apriltag_relocalizer auto_relocalization.launch.py
```
## Conclusion
The apriltag_relocalizer package delivers a lightweight, efficient, and reliable solution for continuous drift correction in indoor autonomous mobile robots using AprilTag fiducial markers.

By seamlessly integrating with ROS2, apriltag_ros detection, and Navigation2 (Nav2) AMCL, it automatically publishes accurate pose corrections whenever known tags enter the camera's field of view—effectively eliminating accumulated odometry errors from wheel slippage, uneven floors, or long-distance travel.

Key strengths include:

Simple configuration via static tag maps
- Real-time performance on embedded hardware
- Easy deployment alongside existing AprilTag detection pipelines
- Proven effectiveness in reducing localization drift without requiring additional sensors

This approach provides a robust, low-cost method for maintaining high-precision localization in GPS-denied environments like warehouses, factories, or labs—making it an essential tool for production-grade AMR systems.