// // #ifndef APRILTAG_RELOCALIZER_HPP_
// // #define APRILTAG_RELOCALIZER_HPP_

// // #include <rclcpp/rclcpp.hpp>
// // #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// // #include <std_msgs/msg/bool.hpp>
// // #include <tf2_ros/transform_listener.h>
// // #include <tf2_ros/buffer.h>
// // #include <tf2/LinearMath/Quaternion.h>

// // class AprilTagRelocalizer : public rclcpp::Node {
// // public:
// //   AprilTagRelocalizer();

// // private:
// //   void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
// //   void drift_check_timer_callback();
// //   void control_loop_callback();
// //   bool publish_robot_pose();

// //   // Publishers
// //   rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nomotion_update_pub_;

// //   // Subscribers
// //   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

// //   // Timers
// //   rclcpp::TimerBase::SharedPtr drift_check_timer_;
// //   rclcpp::TimerBase::SharedPtr control_loop_timer_;

// //   // TF
// //   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
// //   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

// //   // Parameters
// //   double confidence_threshold_;
// //   double log_throttle_;
// //   double pose_publish_delay_;

// //   // State
// //   bool localization_lost_ = false;
// //   bool pose_published_ = false;
// //   double amcl_confidence_;
// //   geometry_msgs::msg::PoseWithCovarianceStamped last_correct_pose_;
// //   geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_amcl_pose_;
// //   rclcpp::Time last_log_time_;
// //   rclcpp::Time last_pose_publish_time_;
// // };

// // #endif // APRILTAG_RELOCALIZER_HPP_



















// // #ifndef APRILTAG_RELOCALIZER_HPP_
// // #define APRILTAG_RELOCALIZER_HPP_

// // #include <rclcpp/rclcpp.hpp>
// // #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// // #include <geometry_msgs/msg/twist.hpp>
// // #include <std_msgs/msg/bool.hpp>
// // #include <apriltag_msgs/msg/april_tag_detection_array.hpp>
// // #include <tf2_ros/buffer.h>
// // #include <tf2_ros/transform_listener.h>
// // #include <vector>

// // class AprilTagRelocalizer : public rclcpp::Node {
// // public:
// //   AprilTagRelocalizer();

// // private:
// //   void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
// //   void tags_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
// //   void drift_check_timer_callback();
// //   void control_loop_callback();
// //   bool publish_robot_pose();
// //   void publish_cmd(double lin, double ang);

// //   // Publishers
// //   rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
// //   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;

// //   // Subscribers
// //   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
// //   rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr tags_sub_;

// //   // Timers
// //   rclcpp::TimerBase::SharedPtr drift_check_timer_;
// //   rclcpp::TimerBase::SharedPtr control_loop_timer_;

// //   // TF2
// //   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
// //   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
// //   geometry_msgs::msg::TransformStamped tag_to_camera_transform_;

// //   // Parameters
// //   double tag_size_;
// //   double search_rot_speed_;
// //   std::vector<double> robot_pose_offset_;
// //   double confidence_threshold_;
// //   double log_throttle_;
// //   double pose_publish_delay_;
// //   double max_tag_z_;
// //   double min_decision_margin_;
// //   int pose_average_count_;
// //   double initial_wait_time_;

// //   // State variables
// //   rclcpp::Time last_log_time_;
// //   rclcpp::Time last_pose_publish_time_;
// //   rclcpp::Time node_start_time_;
// //   rclcpp::Time drift_start_time_;
// //   double amcl_confidence_;
// //   geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_amcl_pose_;
// //   std::vector<std::vector<double>> pose_buffer_; // Stores [x, y, yaw]
// //   int pose_buffer_index_;
// //   bool pose_buffer_filled_;
// //   bool pose_published_;
// //   bool localization_lost_;
// //   bool tag_visible_;
// //   bool first_tag_locked_;
// //   int first_tag_id_;
// //   std::vector<double> stored_position_; // Stores [x, y, yaw]
// //   bool position_stored_;
// // };

// // #endif // APRILTAG_RELOCALIZER_HPP_















// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_eigen/tf2_eigen.hpp>

// #include <geometry_msgs/msg/twist.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <apriltag/apriltag.h>
// #include <apriltag/tag36h11.h>

// #include <Eigen/Dense>
// #include <memory>
// #include <string>
// #include <map>
// #include <array>

// struct TagInfo {
//     double x, y, yaw;
//     double offset_x, offset_y, offset_yaw;
// };

// class AprilTagDriftRelocalizer : public rclcpp::Node
// {
// public:
//     explicit AprilTagDriftRelocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
//     ~AprilTagDriftRelocalizer() override;

//     void stop();

// private:
//     // === Core ===
//     void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
//     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
//     void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
//     void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

//     void drift_check_timer();
//     void control_loop();
//     void publish_corrected_pose(int tag_id, const cv::Mat& rvec, const cv::Mat& tvec);
//     void publish_cmd(double lin, double ang);
//     void publish_marker(int tag_id, double mx, double my, double myaw);

//     double check_amcl();
//     double check_tf();
//     double check_lidar();

//     // === Helpers ===
//     static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);
//     static Eigen::Matrix3d orthonormalize_rotation(const Eigen::Matrix3d& R);
//     static Eigen::Isometry3d rvec_tvec_to_transform(const cv::Mat& rvec, const cv::Mat& tvec);

//     // === TF ===
//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     // === AprilTag ===
//     apriltag_family_t* tf_;
//     apriltag_detector_t* td_;

//     // === Camera ===
//     cv::Mat camera_matrix_;
//     cv::Mat dist_coeffs_;

//     // === Publishers ===
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

//     // === Subscribers ===
//     rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//     rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

//     // === Timers ===
//     rclcpp::TimerBase::SharedPtr drift_timer_;
//     rclcpp::TimerBase::SharedPtr control_timer_;

//     // === State ===
//     std::string state_ = "IDLE";
//     bool localization_lost_ = false;
//     bool relocalization_done_ = false;
//     rclcpp::Time last_log_time_;

//     // === Messages ===
//     geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr latest_amcl_pose_;
//     nav_msgs::msg::Odometry::SharedPtr latest_odom_;
//     sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
//     nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;

//     // === Config ===
//     static constexpr double TAG_SIZE = 0.15;
//     static constexpr double SEARCH_ROT_SPEED = 0.15;
//     static constexpr double SAFE_DIST = 0.6;
//     static constexpr bool STARTUP_FORCE_RELOCALIZE = true;
//     static inline const Eigen::Vector3d CAMERA_OFFSET = Eigen::Vector3d(0.050, 0.0, 0.171);

//     // Drift thresholds
//     static constexpr double AMCL_COV_X_THRESH = 0.03;
//     static constexpr double AMCL_COV_Y_THRESH = 0.03;
//     static constexpr double AMCL_COV_YAW_THRESH = 0.08;
//     static constexpr double RELOCALIZE_SCORE_THRESH = 0.45;
//     static inline const std::map<std::string, double> RELOCALIZE_SCORE_WEIGHTS = {
//         {"amcl", 0.4}, {"tf", 0.3}, {"lidar", 0.3}
//     };

//     // Multi-tag map
//     std::map<int, TagInfo> tag_map_;
//     void load_tag_config();
// };












#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <map>

struct TagInfo {
    double x, y, yaw;
    double offset_x, offset_y, offset_yaw;
};

class AprilTagEKFRelocalizer : public rclcpp::Node
{
public:
    explicit AprilTagEKFRelocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~AprilTagEKFRelocalizer() override;

private:
    // === Core ===
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void publish_ekf_pose(int tag_id, const cv::Mat& rvec, const cv::Mat& tvec);
    void publish_marker(int tag_id, double mx, double my, double myaw);

    // === Helpers ===
    static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);
    static Eigen::Matrix3d orthonormalize_rotation(const Eigen::Matrix3d& R);
    static Eigen::Isometry3d rvec_tvec_to_transform(const cv::Mat& rvec, const cv::Mat& tvec);

    // === TF ===
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // === AprilTag ===
    apriltag_family_t* tf_;
    apriltag_detector_t* td_;

    // === Camera ===
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // === Publishers ===
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // === Subscribers ===
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // === Timer (Optional status) ===
    rclcpp::TimerBase::SharedPtr status_timer_;

    // === Config ===
    static constexpr double TAG_SIZE = 0.16;  // 16 cm
    static inline const Eigen::Vector3d CAMERA_OFFSET = Eigen::Vector3d(0.050, 0.0, 0.171);  // base_link → camera

    // Multi-tag map
    std::map<int, TagInfo> tag_map_;
    void load_tag_config();
};