// // #include "apriltag_relocalizer/apriltag_relocalizer.hpp"
// // #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// // #include <tf2/LinearMath/Matrix3x3.h>

// // AprilTagRelocalizer::AprilTagRelocalizer() : Node("apriltag_relocalizer") {
// //   // Declare parameters
// //   this->declare_parameter("confidence_threshold", 0.7);
// //   this->declare_parameter("log_throttle", 2.0);
// //   this->declare_parameter("pose_publish_delay", 1.0);

// //   // Get parameters
// //   this->get_parameter("confidence_threshold", confidence_threshold_);
// //   this->get_parameter("log_throttle", log_throttle_);
// //   this->get_parameter("pose_publish_delay", pose_publish_delay_);

// //   // Initialize publishers
// //   pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
// //   nomotion_update_pub_ = this->create_publisher<std_msgs::msg::Bool>("/request_nomotion_update", 10);

// //   // Initialize subscribers
// //   amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
// //       "/amcl_pose", 10, std::bind(&AprilTagRelocalizer::amcl_callback, this, std::placeholders::_1));

// //   // Initialize timers
// //   drift_check_timer_ = this->create_wall_timer(
// //       std::chrono::milliseconds(500), std::bind(&AprilTagRelocalizer::drift_check_timer_callback, this));
// //   control_loop_timer_ = this->create_wall_timer(
// //       std::chrono::milliseconds(100), std::bind(&AprilTagRelocalizer::control_loop_callback, this));

// //   // Initialize TF
// //   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));
// //   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// //   // Initialize states
// //   last_log_time_ = this->get_clock()->now();
// //   last_pose_publish_time_ = this->get_clock()->now();
// //   amcl_confidence_ = 1.0;
// //   localization_lost_ = false;
// //   pose_published_ = false;

// //   RCLCPP_INFO(this->get_logger(), "[Node] LiDAR-based drift relocalizer started.");
// // }

// // void AprilTagRelocalizer::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
// //   latest_amcl_pose_ = msg;
// //   try {
// //     auto cov = msg->pose.covariance;
// //     double cov_trace = cov[0] + cov[7] + cov[35];
// //     amcl_confidence_ = std::max(0.0, std::min(1.0, 1.0 - cov_trace));
// //     if (amcl_confidence_ > confidence_threshold_ && !localization_lost_) {
// //       last_correct_pose_ = *msg; // Store pose when confidence is high
// //       tf2::Quaternion q;
// //       tf2::fromMsg(msg->pose.pose.orientation, q);
// //       double roll, pitch, yaw;
// //       tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
// //       RCLCPP_INFO(this->get_logger(), "[AMCL] Stored correct pose: x=%.2f, y=%.2f, yaw=%.2f",
// //                   msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
// //     }
// //   } catch (const std::exception &e) {
// //     RCLCPP_WARN(this->get_logger(), "[AMCL] Covariance error: %s", e.what());
// //     amcl_confidence_ = 0.0;
// //   }
// // }

// // void AprilTagRelocalizer::drift_check_timer_callback() {
// //   if (amcl_confidence_ < confidence_threshold_ && !localization_lost_) {
// //     localization_lost_ = true;
// //     pose_published_ = false;
// //     RCLCPP_WARN(this->get_logger(), "[DRIFT DETECTED] AMCL confidence=%.2f, threshold=%.2f → initiating relocalization",
// //                 amcl_confidence_, confidence_threshold_);
// //   }
// // }

// // void AprilTagRelocalizer::control_loop_callback() {
// //   if (!localization_lost_) {
// //     if (amcl_confidence_ > confidence_threshold_ && pose_published_) {
// //       pose_published_ = false;
// //       RCLCPP_INFO(this->get_logger(), "[Control Loop] Localization restored, ready for next drift.");
// //     }
// //     return;
// //   }

// //   auto now = this->get_clock()->now();
// //   if ((now - last_pose_publish_time_).seconds() < pose_publish_delay_) {
// //     return;
// //   }

// //   if (publish_robot_pose()) {
// //     pose_published_ = true;
// //     localization_lost_ = false;
// //     last_pose_publish_time_ = now;
// //     RCLCPP_INFO(this->get_logger(), "[Relocalization] Pose published successfully.");
// //   } else {
// //     RCLCPP_WARN(this->get_logger(), "[Relocalization] Failed to publish pose, triggering AMCL global localization.");
// //     std_msgs::msg::Bool msg;
// //     msg.data = true;
// //     nomotion_update_pub_->publish(msg);
// //   }
// // }

// // bool AprilTagRelocalizer::publish_robot_pose() {
// //   if (pose_published_ || !last_correct_pose_.header.frame_id.empty()) {
// //     // Publish the last correct pose
// //     auto pose = last_correct_pose_;
// //     pose.header.stamp = this->get_clock()->now(); // Update timestamp
// //     pose_pub_->publish(pose);
// //     tf2::Quaternion q;
// //     tf2::fromMsg(pose.pose.pose.orientation, q);
// //     double roll, pitch, yaw;
// //     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
// //     RCLCPP_INFO(this->get_logger(), "[InitialPose Published] X=%.2f, Y=%.2f, Yaw=%.2f",
// //                 pose.pose.pose.position.x, pose.pose.pose.position.y, yaw);
// //     return true;
// //   } else {
// //     RCLCPP_WARN(this->get_logger(), "[Publish Pose] No valid previous pose available.");
// //     return false;
// //   }
// // }

// // int main(int argc, char *argv[]) {
// //   rclcpp::init(argc, argv);
// //   auto node = std::make_shared<AprilTagRelocalizer>();
// //   rclcpp::spin(node);
// //   rclcpp::shutdown();
// //   return 0;
// // }













// #include "apriltag_relocalizer/apriltag_relocalizer.hpp"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Transform.h>
// #include <numeric>
// #include <cmath>

// AprilTagRelocalizer::AprilTagRelocalizer() : Node("apriltag_relocalizer") {
//   // Declare parameters
//   this->declare_parameter("tag_size", 0.0762);
//   this->declare_parameter("search_rot_speed", 0.05);
//   this->declare_parameter("robot_pose_offset", std::vector<double>{0.0, 0.0, 0.0});
//   this->declare_parameter("confidence_threshold", 0.7);
//   this->declare_parameter("log_throttle", 2.0);
//   this->declare_parameter("pose_publish_delay", 1.0);
//   this->declare_parameter("max_tag_z", 5.0);
//   this->declare_parameter("min_decision_margin", 50.0);
//   this->declare_parameter("pose_average_count", 1);
//   this->declare_parameter("initial_wait_time", 15.0);

//   // Get parameters
//   this->get_parameter("tag_size", tag_size_);
//   this->get_parameter("search_rot_speed", search_rot_speed_);
//   this->get_parameter("robot_pose_offset", robot_pose_offset_);
//   this->get_parameter("confidence_threshold", confidence_threshold_);
//   this->get_parameter("log_throttle", log_throttle_);
//   this->get_parameter("pose_publish_delay", pose_publish_delay_);
//   this->get_parameter("max_tag_z", max_tag_z_);
//   this->get_parameter("min_decision_margin", min_decision_margin_);
//   this->get_parameter("pose_average_count", pose_average_count_);
//   this->get_parameter("initial_wait_time", initial_wait_time_);

//   // Initialize publishers
//   pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
//   cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//   status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/localization_status", 10);

//   // Initialize subscribers
//   amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//     "/amcl_pose", 10, std::bind(&AprilTagRelocalizer::amcl_callback, this, std::placeholders::_1));
//   tags_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
//     "/detections", 10, std::bind(&AprilTagRelocalizer::tags_callback, this, std::placeholders::_1));

//   // Initialize timers
//   drift_check_timer_ = this->create_wall_timer(
//     std::chrono::milliseconds(500), std::bind(&AprilTagRelocalizer::drift_check_timer_callback, this));
//   control_loop_timer_ = this->create_wall_timer(
//     std::chrono::milliseconds(100), std::bind(&AprilTagRelocalizer::control_loop_callback, this));

//   // Initialize TF with larger buffer
//   tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(30.0));
//   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//   // Initialize states
//   last_log_time_ = this->get_clock()->now();
//   last_pose_publish_time_ = this->get_clock()->now();
//   node_start_time_ = this->get_clock()->now();
//   drift_start_time_ = rclcpp::Time(0);
//   amcl_confidence_ = 1.0;
//   pose_buffer_.resize(pose_average_count_, {0.0, 0.0, 0.0});
//   pose_buffer_index_ = 0;
//   pose_buffer_filled_ = false;
//   pose_published_ = false;
//   localization_lost_ = false;
//   tag_visible_ = false;
//   first_tag_locked_ = false;
//   position_stored_ = false;

//   RCLCPP_INFO(this->get_logger(), "[Node] AprilTag drift relocalizer started.");
// }

// void AprilTagRelocalizer::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
//   latest_amcl_pose_ = msg;
//   try {
//     auto cov = msg->pose.covariance;
//     double cov_trace = cov[0] + cov[7] + cov[35];
//     amcl_confidence_ = std::max(0.0, std::min(1.0, 1.0 - cov_trace));
//     RCLCPP_INFO(this->get_logger(), "[AMCL] Confidence=%.2f, Position: x=%.2f, y=%.2f",
//                 amcl_confidence_, msg->pose.pose.position.x, msg->pose.pose.position.y);
//     if (amcl_confidence_ >= confidence_threshold_ && !localization_lost_) {
//       tf2::Quaternion q;
//       tf2::fromMsg(msg->pose.pose.orientation, q);
//       double roll, pitch, yaw;
//       tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//       while (yaw > M_PI) yaw -= 2 * M_PI;
//       while (yaw < -M_PI) yaw += 2 * M_PI;
//       stored_position_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw};
//       position_stored_ = true;
//       RCLCPP_INFO(this->get_logger(), "[AMCL] Stored position: x=%.2f, y=%.2f, yaw=%.2f",
//                   stored_position_[0], stored_position_[1], stored_position_[2]);
//     } else {
//       RCLCPP_DEBUG(this->get_logger(), "[AMCL] Not storing position: confidence=%.2f (threshold=%.2f), localization_lost_=%s",
//                    amcl_confidence_, confidence_threshold_, localization_lost_ ? "true" : "false");
//     }
//   } catch (const std::exception &e) {
//     RCLCPP_WARN(this->get_logger(), "[AMCL] Covariance error: %s", e.what());
//     amcl_confidence_ = 0.0;
//   }
// }

// void AprilTagRelocalizer::tags_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
//   if (!localization_lost_ || pose_published_) {
//     return;
//   }
//   tag_visible_ = false;
//   for (const auto &detection : msg->detections) {
//     if (!first_tag_locked_) {
//       first_tag_id_ = detection.id;
//       first_tag_locked_ = true;
//       RCLCPP_INFO(this->get_logger(), "[Tag] Locked to tag ID %d.", first_tag_id_);
//     }
//     if (detection.id == first_tag_id_) {
//       RCLCPP_INFO(this->get_logger(), "[Tag Debug] Detection ID %d: family=%s, centre=(%.2f, %.2f), decision_margin=%.2f",
//                   detection.id, detection.family.c_str(),
//                   detection.centre.x, detection.centre.y, detection.decision_margin);
//       if (detection.decision_margin < min_decision_margin_) {
//         RCLCPP_WARN(this->get_logger(), "[Tag] Low decision_margin=%.2f for tag %d, ignoring detection.",
//                     detection.decision_margin, first_tag_id_);
//         continue;
//       }
//       try {
//         tag_to_camera_transform_ = tf_buffer_->lookupTransform(
//           "camera_link_optical", "tag36h11_" + std::to_string(first_tag_id_),
//           tf2::TimePointZero, tf2::durationFromSec(5.0));
//         double z = tag_to_camera_transform_.transform.translation.z;
//         tf2::Quaternion q_camera_to_tag;
//         tf2::fromMsg(tag_to_camera_transform_.transform.rotation, q_camera_to_tag);
//         double roll_ct, pitch_ct, yaw_ct;
//         tf2::Matrix3x3(q_camera_to_tag).getRPY(roll_ct, pitch_ct, yaw_ct);
//         RCLCPP_INFO(this->get_logger(), "[TF Debug] camera_to_tag: x=%.2f, y=%.2f, z=%.2f, q=[%.2f, %.2f, %.2f, %.2f], yaw=%.2f",
//                     tag_to_camera_transform_.transform.translation.x,
//                     tag_to_camera_transform_.transform.translation.y, z,
//                     q_camera_to_tag.x(), q_camera_to_tag.y(), q_camera_to_tag.z(), q_camera_to_tag.w(), yaw_ct);
//         if (std::abs(z) > max_tag_z_) {
//           RCLCPP_WARN(this->get_logger(), "[Tag] Invalid z=%.2f for tag %d, ignoring detection.", z, first_tag_id_);
//           continue;
//         }
//         tag_visible_ = true;
//         RCLCPP_INFO(this->get_logger(), "[Tag] Tag %d visible.", first_tag_id_);
//       } catch (const tf2::TransformException &e) {
//         RCLCPP_WARN(this->get_logger(), "[TF] Lookup error for camera_to_tag: %s", e.what());
//       }
//       break;
//     }
//   }
//   if (!tag_visible_ && first_tag_locked_) {
//     RCLCPP_INFO(this->get_logger(), "[Tag] Lost tag %d.", first_tag_id_);
//   }
// }

// void AprilTagRelocalizer::drift_check_timer_callback() {
//   auto now = this->get_clock()->now();
//   if (amcl_confidence_ < confidence_threshold_ && !localization_lost_) {
//     if ((now - node_start_time_).seconds() > initial_wait_time_ || position_stored_) {
//       localization_lost_ = true;
//       pose_published_ = false;
//       first_tag_locked_ = false;
//       pose_buffer_.clear();
//       pose_buffer_.resize(pose_average_count_, {0.0, 0.0, 0.0});
//       pose_buffer_index_ = 0;
//       pose_buffer_filled_ = false;
//       drift_start_time_ = now; // Store drift start time
//       publish_cmd(0.0, 0.0);
//       RCLCPP_WARN(this->get_logger(), "[DRIFT DETECTED] AMCL confidence=%.2f, threshold=%.2f → searching tag",
//                   amcl_confidence_, confidence_threshold_);
//     } else {
//       RCLCPP_INFO(this->get_logger(), "[Drift Check] Waiting for high-confidence AMCL pose...");
//     }
//   }
// }

// void AprilTagRelocalizer::control_loop_callback() {
//   if (!localization_lost_ || pose_published_) {
//     if (amcl_confidence_ > confidence_threshold_ && pose_published_) {
//       pose_published_ = false;
//       first_tag_locked_ = false;
//       RCLCPP_INFO(this->get_logger(), "[Control Loop] Localization restored, ready for next drift.");
//     }
//     return;
//   }
//   if (!tag_visible_) {
//     publish_cmd(0.0, search_rot_speed_);
//     auto now = this->get_clock()->now();
//     if ((now - last_log_time_).seconds() > log_throttle_) {
//       RCLCPP_INFO(this->get_logger(), "[Control Loop] Rotating to find tag...");
//       last_log_time_ = now;
//     }
//     return;
//   }
//   auto now = this->get_clock()->now();
//   if ((now - last_pose_publish_time_).seconds() < pose_publish_delay_) {
//     return;
//   }
//   if (publish_robot_pose()) {
//     pose_published_ = true;
//     localization_lost_ = false;
//     publish_cmd(0.0, 0.0);
//     last_pose_publish_time_ = now;
//     RCLCPP_INFO(this->get_logger(), "[Relocalization] Pose published successfully.");
//   } else {
//     RCLCPP_WARN(this->get_logger(), "[Relocalization] Failed to publish pose.");
//   }
// }

// bool AprilTagRelocalizer::publish_robot_pose() {
//   if (pose_published_) {
//     RCLCPP_WARN(this->get_logger(), "[Relocalization] Pose already published.");
//     return false;
//   }
//   if (!position_stored_) {
//     RCLCPP_WARN(this->get_logger(), "[Relocalization] No stored position available before drift, cannot publish pose.");
//     return false;
//   }
//   // Store pre-drift x, y, yaw
//   pose_buffer_[pose_buffer_index_] = stored_position_;
//   pose_buffer_index_ = (pose_buffer_index_ + 1) % pose_average_count_;
//   if (pose_buffer_index_ == 0) {
//     pose_buffer_filled_ = true;
//   }
//   if (!pose_buffer_filled_) {
//     RCLCPP_INFO(this->get_logger(), "[Relocalization] Pose buffer not filled, waiting for more detections.");
//     return false;
//   }
//   // Average x, y from pre-drift poses
//   double avg_x = 0.0, avg_y = 0.0;
//   for (const auto &pos : pose_buffer_) {
//     avg_x += pos[0];
//     avg_y += pos[1];
//   }
//   avg_x /= pose_average_count_;
//   avg_y /= pose_average_count_;
//   // Calculate turning angle
//   auto now = this->get_clock()->now();
//   double turning_angle = search_rot_speed_ * (now - drift_start_time_).seconds();
//   // Use pre-drift yaw + turning angle
//   double pre_drift_yaw = stored_position_[2];
//   double final_yaw = pre_drift_yaw + turning_angle;
//   while (final_yaw > M_PI) final_yaw -= 2 * M_PI;
//   while (final_yaw < -M_PI) final_yaw += 2 * M_PI;
//   // Apply robot_pose_offset
//   final_yaw += robot_pose_offset_[2];
//   while (final_yaw > M_PI) final_yaw -= 2 * M_PI;
//   while (final_yaw < -M_PI) final_yaw += 2 * M_PI;
//   tf2::Quaternion q_new;
//   q_new.setRPY(0.0, 0.0, final_yaw);
//   auto pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
//   pose->header.frame_id = "map";
//   pose->header.stamp = this->get_clock()->now();
//   pose->pose.pose.position.x = avg_x;
//   pose->pose.pose.position.y = avg_y;
//   pose->pose.pose.position.z = 0.0;
//   pose->pose.pose.orientation = tf2::toMsg(q_new);
//   pose->pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
//                            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
//                            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
//                            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
//                            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
//                            0.0, 0.0, 0.0, 0.0, 0.0, 0.05};
//   pose_pub_->publish(*pose);
//   RCLCPP_INFO(this->get_logger(), "[InitialPose Published] X=%.2f, Y=%.2f, Yaw=%.2f (pre_drift_yaw=%.2f, turning_angle=%.2f)",
//               avg_x, avg_y, final_yaw, pre_drift_yaw, turning_angle);
//   pose_buffer_.clear();
//   pose_buffer_.resize(pose_average_count_, {0.0, 0.0, 0.0});
//   pose_buffer_index_ = 0;
//   pose_buffer_filled_ = false;
//   return true;
// }

// void AprilTagRelocalizer::publish_cmd(double lin, double ang) {
//   geometry_msgs::msg::Twist msg;
//   msg.linear.x = lin;
//   msg.angular.z = ang;
//   cmd_pub_->publish(msg);
// }

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<AprilTagRelocalizer>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

















// #include "apriltag_relocalizer/apriltag_relocalizer.hpp"
// #include <rclcpp/rclcpp.hpp>
// #include <yaml-cpp/yaml.h>
// #include <ament_index_cpp/get_package_share_directory.hpp>

// using namespace std::chrono_literals;

// // === Load Tags ===
// void AprilTagDriftRelocalizer::load_tag_config()
// {
//     std::string config_file = this->declare_parameter<std::string>("tag_config", "config/tags.yaml");
//     std::string full_path = ament_index_cpp::get_package_share_directory("apriltag_relocalizer") + "/" + config_file;

//     try {
//         YAML::Node config = YAML::LoadFile(full_path);
//         for (const auto& tag_node : config["tags"]) {
//             int id = tag_node.first.as<int>();
//             auto& t = tag_node.second;
//             tag_map_[id] = {
//                 t["x"].as<double>(),
//                 t["y"].as<double>(),
//                 t["yaw"].as<double>(),
//                 t["offset_x"].as<double>(0.0),
//                 t["offset_y"].as<double>(0.0),
//                 t["offset_yaw"].as<double>(0.0)
//             };
//             RCLCPP_INFO(this->get_logger(), "Loaded tag %d: (%.3f, %.3f, %.3f)", id,
//                         tag_map_[id].x, tag_map_[id].y, tag_map_[id].yaw);
//         }
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(this->get_logger(), "Failed to load tag config: %s", e.what());
//     }
// }

// // === Constructor ===
// AprilTagDriftRelocalizer::AprilTagDriftRelocalizer(const rclcpp::NodeOptions & options)
// : Node("apriltag_drift_relocalizer", options)
// {
//     RCLCPP_INFO(this->get_logger(), "AprilTag drift relocalizer started.");
//     load_tag_config();

//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     tf_ = tag36h11_create();
//     td_ = apriltag_detector_create();
//     apriltag_detector_add_family(td_, tf_);

//     cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//     pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
//     status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/localization_status", 10);
//     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tag_markers", 10);

//     camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
//         "/camera/camera_info", 10,
//         std::bind(&AprilTagDriftRelocalizer::camera_info_callback, this, std::placeholders::_1));

//     image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//         "/camera/image_raw", 10,
//         std::bind(&AprilTagDriftRelocalizer::image_callback, this, std::placeholders::_1));

//     amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
//         "/amcl_pose", 10,
//         std::bind(&AprilTagDriftRelocalizer::amcl_callback, this, std::placeholders::_1));

//     odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         "/odom", 10,
//         std::bind(&AprilTagDriftRelocalizer::odom_callback, this, std::placeholders::_1));

//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan", 10,
//         std::bind(&AprilTagDriftRelocalizer::scan_callback, this, std::placeholders::_1));

//     map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/map", 10,
//         std::bind(&AprilTagDriftRelocalizer::map_callback, this, std::placeholders::_1));

//     drift_timer_ = this->create_wall_timer(500ms, std::bind(&AprilTagDriftRelocalizer::drift_check_timer, this));
//     control_timer_ = this->create_wall_timer(100ms, std::bind(&AprilTagDriftRelocalizer::control_loop, this));

//     last_log_time_ = this->now();

//     if (STARTUP_FORCE_RELOCALIZE) {
//         localization_lost_ = true;
//         state_ = "SEARCH_MARKER";
//         RCLCPP_WARN(this->get_logger(), "Startup relocalization enabled. Searching for AprilTags...");
//     }
// }

// AprilTagDriftRelocalizer::~AprilTagDriftRelocalizer()
// {
//     apriltag_detector_destroy(td_);
//     tag36h11_destroy(tf_);
// }

// // === Callbacks ===
// void AprilTagDriftRelocalizer::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
// {
//     if (camera_matrix_.empty()) {
//         camera_matrix_ = cv::Mat(3, 3, CV_64F);
//         for (int i = 0; i < 9; ++i) camera_matrix_.at<double>(i/3, i%3) = msg->k[i];
//         dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
//         for (size_t i = 0; i < msg->d.size(); ++i) dist_coeffs_.at<double>(i) = msg->d[i];
//         RCLCPP_INFO(this->get_logger(), "Camera calibration received.");
//     }
// }

// void AprilTagDriftRelocalizer::amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { latest_amcl_pose_ = msg; }
// void AprilTagDriftRelocalizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; }
// void AprilTagDriftRelocalizer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; }
// void AprilTagDriftRelocalizer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_msg_ = msg; }

// // === Drift Check ===
// void AprilTagDriftRelocalizer::drift_check_timer()
// {
//     double amcl_score = check_amcl();
//     double tf_score = check_tf();
//     double lidar_score = check_lidar();

//     double combined = amcl_score * RELOCALIZE_SCORE_WEIGHTS.at("amcl") +
//                       tf_score * RELOCALIZE_SCORE_WEIGHTS.at("tf") +
//                       lidar_score * RELOCALIZE_SCORE_WEIGHTS.at("lidar");

//     std_msgs::msg::Bool status;
//     status.data = (combined > RELOCALIZE_SCORE_THRESH);
//     status_pub_->publish(status);

//     if (combined > RELOCALIZE_SCORE_THRESH && !localization_lost_ && !relocalization_done_ ) {
//         localization_lost_ = true;
//         state_ = "SEARCH_MARKER";
//         publish_cmd(0.0, 0.0);
//         RCLCPP_WARN(this->get_logger(), "[DRIFT DETECTED] Score=%.2f. Searching for AprilTags...", combined);
//     }
// }

// double AprilTagDriftRelocalizer::check_amcl()
// {
//     if (!latest_amcl_pose_) return 0.0;
//     auto& cov = latest_amcl_pose_->pose.covariance;
//     double sx = cov[0] / AMCL_COV_X_THRESH;
//     double sy = cov[7] / AMCL_COV_Y_THRESH;
//     double syaw = cov[35] / AMCL_COV_YAW_THRESH;
//     return std::min(1.0, (sx + sy + syaw) / 3.0);
// }

// double AprilTagDriftRelocalizer::check_tf()
// {
//     try {
//         auto t = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
//         return std::min(1.0, std::hypot(t.transform.translation.x, t.transform.translation.y) / 0.25);
//     } catch (...) { return 0.0; }
// }

// double AprilTagDriftRelocalizer::check_lidar()
// {
//     if (!latest_scan_) return 0.0;
//     std::vector<float> front_ranges;
//     front_ranges.insert(front_ranges.end(), latest_scan_->ranges.begin(), latest_scan_->ranges.begin() + 10);
//     front_ranges.insert(front_ranges.end(), latest_scan_->ranges.end() - 10, latest_scan_->ranges.end());

//     std::vector<float> valid;
//     for (float r : front_ranges) if (std::isfinite(r)) valid.push_back(r);
//     if (valid.empty()) return 0.0;

//     float min_dist = *std::min_element(valid.begin(), valid.end());
//     return min_dist < SAFE_DIST ? std::min(1.0, (SAFE_DIST - min_dist) / SAFE_DIST) : 0.0;
// }

// // === Image Callback ===
// void AprilTagDriftRelocalizer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
// {
//     if (!localization_lost_ || camera_matrix_.empty()) return;

//     cv_bridge::CvImagePtr cv_ptr;
//     try { cv_ptr = cv_bridge::toCvCopy(msg, "bgr8"); } catch (...) { return; }

//     cv::Mat gray;
//     cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
//     image_u8_t im{static_cast<int>(gray.cols), static_cast<int>(gray.rows), static_cast<int>(gray.cols), gray.data};
//     zarray_t *detections = apriltag_detector_detect(td_, &im);

//     double best_dist = std::numeric_limits<double>::max();
//     apriltag_detection_t *best_det = nullptr;
//     int best_id = -1;

//     for (int i = 0; i < zarray_size(detections); ++i) {
//         apriltag_detection_t *det;
//         zarray_get(detections, i, &det);
//         if (tag_map_.find(det->id) != tag_map_.end()) {
//             double cx = det->c[0], cy = det->c[1];
//             double dist = std::hypot(cx - gray.cols/2.0, cy - gray.rows/2.0);
//             if (dist < best_dist) {
//                 best_dist = dist;
//                 best_det = det;
//                 best_id = det->id;
//             }
//         }
//     }

//     if (best_det) {
//         std::vector<cv::Point3d> obj_pts = {
//             {-TAG_SIZE/2, -TAG_SIZE/2, 0}, {TAG_SIZE/2, -TAG_SIZE/2, 0},
//             {TAG_SIZE/2, TAG_SIZE/2, 0}, {-TAG_SIZE/2, TAG_SIZE/2, 0}
//         };
//         std::vector<cv::Point2d> img_pts;
//         for (int i = 0; i < 4; ++i) img_pts.emplace_back(best_det->p[i][0], best_det->p[i][1]);

//         cv::Mat rvec, tvec;
//         cv::solvePnP(obj_pts, img_pts, camera_matrix_, dist_coeffs_, rvec, tvec);

//         publish_corrected_pose(best_id, rvec, tvec);
//         publish_marker(best_id, tag_map_[best_id].x, tag_map_[best_id].y, tag_map_[best_id].yaw);

//         relocalization_done_ = true;          // ← STOP FOREVER
//         localization_lost_ = false;
//         state_ = "IDLE";
//         publish_cmd(0.0, 0.0);
//         RCLCPP_INFO(this->get_logger(), "Relocalized using tag %d - ONE SHOT DONE", best_id);
//     }

//     apriltag_detections_destroy(detections);
// }

// // === Pose Publishing ===
// void AprilTagDriftRelocalizer::publish_corrected_pose(int tag_id, const cv::Mat& rvec, const cv::Mat& tvec)
// {
//     const auto& tag = tag_map_[tag_id];
//     double mx = tag.x, my = tag.y, myaw = tag.yaw;

//     Eigen::Matrix3d R_map_tag;
//     R_map_tag << std::cos(myaw), -std::sin(myaw), 0,
//                  std::sin(myaw),  std::cos(myaw), 0,
//                  0,              0,             1;

//     Eigen::Isometry3d T_map_tag = Eigen::Isometry3d::Identity();
//     T_map_tag.linear() = R_map_tag;
//     T_map_tag.translation() = Eigen::Vector3d(mx, my, 0.0);

//     Eigen::Isometry3d T_tag_cam = rvec_tvec_to_transform(rvec, tvec).inverse();
//     Eigen::Isometry3d T_cam_base = Eigen::Isometry3d::Identity();
//     T_cam_base.translation() = CAMERA_OFFSET;

//     Eigen::Isometry3d T_map_base = T_map_tag * T_tag_cam * T_cam_base;

//     double rx = T_map_base.translation().x() + tag.offset_x;
//     double ry = T_map_base.translation().y() + tag.offset_y;
//     double yaw = std::atan2(T_map_base.linear()(1,0), T_map_base.linear()(0,0)) + tag.offset_yaw;

//     auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
//     pose_msg->header.frame_id = "map";
//     pose_msg->header.stamp = this->now();
//     pose_msg->pose.pose.position.x = rx;
//     pose_msg->pose.pose.position.y = ry;
//     pose_msg->pose.pose.orientation = quaternion_from_yaw(yaw);
//     std::fill(pose_msg->pose.covariance.begin(), pose_msg->pose.covariance.end(), 0.05);

//     pose_pub_->publish(*pose_msg);
//     RCLCPP_INFO(this->get_logger(), "/initialpose published (tag %d) X=%.2f, Y=%.2f, Yaw=%.2f", tag_id, rx, ry, yaw);
// }

// // === RViz Marker ===
// void AprilTagDriftRelocalizer::publish_marker(int tag_id, double mx, double my, double myaw)
// {
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = "map";
//     marker.header.stamp = this->now();
//     marker.ns = "apriltag";
//     marker.id = tag_id;
//     marker.type = visualization_msgs::msg::Marker::CUBE;
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.pose.position.x = mx; marker.pose.position.y = my; marker.pose.position.z = 0.0;
//     marker.pose.orientation = quaternion_from_yaw(myaw);
//     marker.scale.x = marker.scale.y = 0.15; marker.scale.z = 0.01;
//     marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 0.7;
//     marker.lifetime = rclcpp::Duration::from_seconds(5.0);
//     marker_pub_->publish(marker);
// }

// // === Control ===
// void AprilTagDriftRelocalizer::control_loop()
// {
//     if (state_ == "SEARCH_MARKER") {
//         publish_cmd(0.0, SEARCH_ROT_SPEED);
//         if ((this->now() - last_log_time_) > 2s) {
//             RCLCPP_INFO(this->get_logger(), "Searching for AprilTags...");
//             last_log_time_ = this->now();
//         }
//     } else {
//         publish_cmd(0.0, 0.0);
//     }
// }

// void AprilTagDriftRelocalizer::publish_cmd(double lin, double ang)
// {
//     auto msg = geometry_msgs::msg::Twist();
//     msg.linear.x = lin; msg.angular.z = ang;
//     cmd_pub_->publish(msg);
// }

// void AprilTagDriftRelocalizer::stop() { publish_cmd(0.0, 0.0); }

// // === Helpers ===
// geometry_msgs::msg::Quaternion AprilTagDriftRelocalizer::quaternion_from_yaw(double yaw)
// {
//     Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
//     geometry_msgs::msg::Quaternion quat;
//     quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();
//     return quat;
// }

// Eigen::Matrix3d AprilTagDriftRelocalizer::orthonormalize_rotation(const Eigen::Matrix3d& R)
// {
//     Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
//     return svd.matrixU() * svd.matrixV().transpose();
// }

// Eigen::Isometry3d AprilTagDriftRelocalizer::rvec_tvec_to_transform(const cv::Mat& rvec, const cv::Mat& tvec)
// {
//     cv::Mat R;
//     cv::Rodrigues(rvec, R);
//     Eigen::Matrix3d rot;
//     for (int i = 0; i < 3; ++i)
//         for (int j = 0; j < 3; ++j)
//             rot(i, j) = R.at<double>(i, j);
//     rot = orthonormalize_rotation(rot);

//     Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//     T.linear() = rot;
//     T.translation() = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
//     return T;
// }

// // === Main ===
// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<AprilTagDriftRelocalizer>();
//     rclcpp::spin(node);
//     node->stop();
//     rclcpp::shutdown();
//     return 0;
// }



















// #include <rclcpp/rclcpp.hpp>
// #include <yaml-cpp/yaml.h>
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <Eigen/Geometry>
// #include <unordered_map>
// #include <chrono>
// #include <string>

// using namespace std::chrono_literals;

// struct TagPose {
//     double x, y, yaw;
// };

// class AprilTagDriftRelocalizer : public rclcpp::Node {
// public:
//     AprilTagDriftRelocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
//     : Node("apriltag_drift_relocalizer", options)
//     {
//         RCLCPP_INFO(this->get_logger(), "AprilTag drift relocalizer started.");

//         // === Parameters ===
//         this->declare_parameter<std::string>("tag_config", "config/tags.yaml");
//         this->declare_parameter<double>("relocalize_check_period", 0.2);
//         this->declare_parameter<double>("imu_yaw_offset", 0.0);
//         this->declare_parameter<double>("search_angular_vel", 0.1);
//         this->declare_parameter<double>("min_relocalize_distance", 1.5);  // NEW: minimum distance

//         imu_yaw_offset_ = this->get_parameter("imu_yaw_offset").as_double();
//         search_angular_vel_ = this->get_parameter("search_angular_vel").as_double();
//         min_relocalize_distance_ = this->get_parameter("min_relocalize_distance").as_double();

//         // === TF setup ===
//         tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//         tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         camera_frame_ = "camera_link_optical";
//         base_frame_ = "base_link";

//         // === Publishers ===
//         cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_tag", 10);
//         pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
//         marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tag_markers", 10);

//         // === IMU subscriber ===
//         imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "/imu/data", 20,
//             std::bind(&AprilTagDriftRelocalizer::imu_callback, this, std::placeholders::_1)
//         );

//         // === Timer for drift check ===
//         double check_period = this->get_parameter("relocalize_check_period").as_double();
//         relocalize_timer_ = this->create_wall_timer(
//             std::chrono::duration<double>(check_period),
//             std::bind(&AprilTagDriftRelocalizer::try_relocalize_from_tf, this)
//         );

//         load_tag_config();
//         localization_lost_ = true;  // start assuming drift
//     }

// private:
//     // === Tag data ===
//     std::unordered_map<int, TagPose> tag_map_;

//     // === TF ===
//     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     // === ROS pubs/subs ===
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//     rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
//     rclcpp::TimerBase::SharedPtr relocalize_timer_;

//     // === Frames & flags ===
//     std::string camera_frame_;
//     std::string base_frame_;
//     bool localization_lost_ = true;
//     bool relocalization_done_ = false;

//     // === IMU data ===
//     double latest_imu_yaw_ = 0.0;
//     bool imu_received_ = false;
//     double imu_yaw_offset_ = 0.0;

//     // === Motion control ===
//     double search_angular_vel_;
//     rclcpp::Time last_rotate_cmd_time_;
//     double min_relocalize_distance_;  // NEW

//     // === Load tag world coordinates ===
//     void load_tag_config()
//     {
//         std::string config_file = this->get_parameter("tag_config").as_string();
//         std::string full_path = ament_index_cpp::get_package_share_directory("apriltag_relocalizer") + "/" + config_file;

//         try {
//             YAML::Node config = YAML::LoadFile(full_path);
//             for (const auto& tag_node : config["tags"]) {
//                 int id = tag_node.first.as<int>();
//                 auto& t = tag_node.second;
//                 tag_map_[id] = {
//                     t["x"].as<double>(),
//                     t["y"].as<double>(),
//                     t["yaw"].as<double>()
//                 };
//                 RCLCPP_INFO(this->get_logger(),
//                     "Loaded tag %d: (x=%.3f, y=%.3f, yaw=%.3f)",
//                     id, tag_map_[id].x, tag_map_[id].y, tag_map_[id].yaw);
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to load tag config: %s", e.what());
//         }
//     }

//     // === IMU callback ===
//     void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
//     {
//         tf2::Quaternion q(
//             msg->orientation.x,
//             msg->orientation.y,
//             msg->orientation.z,
//             msg->orientation.w
//         );

//         double roll, pitch, yaw;
//         tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

//         latest_imu_yaw_ = yaw + imu_yaw_offset_;
//         while (latest_imu_yaw_ > M_PI) latest_imu_yaw_ -= 2 * M_PI;
//         while (latest_imu_yaw_ < -M_PI) latest_imu_yaw_ += 2 * M_PI;

//         imu_received_ = true;
//     }

//     // === Relocalization logic ===
//     void try_relocalize_from_tf()
//     {
//         if (!localization_lost_ || relocalization_done_) return;

//         bool tag_found = false;

//         for (const auto& [tag_id, tag_pose] : tag_map_) {
//             std::string tag_frame = "tag36h11_" + std::to_string(tag_id);
//             geometry_msgs::msg::TransformStamped tf_base_to_tag;

//             try {
//                 tf_base_to_tag = tf_buffer_->lookupTransform(base_frame_, tag_frame, tf2::TimePointZero);
//                 tag_found = true;
//             } catch (const tf2::TransformException & ex) {
//                 continue;
//             }

//             // === Compute distance from robot to tag ===
//             double dx = tf_base_to_tag.transform.translation.x;
//             double dy = tf_base_to_tag.transform.translation.y;
//             double distance = std::sqrt(dx*dx + dy*dy);

//             if (distance > min_relocalize_distance_) {
//                 RCLCPP_INFO(this->get_logger(),
//                     "Tag %d detected but too far (%.2f m > min %.2f m)",
//                     tag_id, distance, min_relocalize_distance_);
//                 continue;  // skip this tag
//             }

//             // === Stop rotation ===
//             stop_rotation();

//             // === Extract base→tag translation ===
//             Eigen::Vector3d t_base_to_tag(dx, dy, tf_base_to_tag.transform.translation.z);

//             // === Compute map→base (robot) position ===
//             double tag_x = tag_pose.x;
//             double tag_y = tag_pose.y;
//             double tag_yaw = tag_pose.yaw;
//             Eigen::Rotation2Dd R_tag_to_map(tag_yaw);

//             Eigen::Vector2d p_base_in_tag(t_base_to_tag.x(), t_base_to_tag.y());
//             Eigen::Vector2d p_base_in_map = Eigen::Vector2d(tag_x, tag_y) - R_tag_to_map * p_base_in_tag;

//             double world_x = p_base_in_map.x();
//             double world_y = p_base_in_map.y();

//             double world_yaw = imu_received_ ? latest_imu_yaw_ : tag_pose.yaw;
//             while (world_yaw > M_PI) world_yaw -= 2 * M_PI;
//             while (world_yaw < -M_PI) world_yaw += 2 * M_PI;

//             RCLCPP_WARN(this->get_logger(),
//                 "RELOCALIZED using tag %d → Robot at (%.3f, %.3f, %.3f rad)",
//                 tag_id, world_x, world_y, world_yaw);

//             publish_marker(tag_id, world_x, world_y);
//             publish_initial_pose(world_x, world_y, world_yaw);

//             relocalization_done_ = true;
//             localization_lost_ = false;
//             break;
//         }

//         if (!tag_found) {
//             rotate_to_search();
//         }
//     }

//     // === Rotate slowly to search for tags ===
//     void rotate_to_search()
//     {
//         geometry_msgs::msg::Twist cmd;
//         cmd.angular.z = search_angular_vel_;
//         cmd_pub_->publish(cmd);
//     }

//     // === Stop rotation ===
//     void stop_rotation()
//     {
//         geometry_msgs::msg::Twist cmd;
//         cmd.angular.z = 0.0;
//         cmd_pub_->publish(cmd);
//     }

//     // === Publish initial pose ===
//     void publish_initial_pose(double x, double y, double yaw)
//     {
//         geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
//         pose_msg.header.frame_id = "map";
//         pose_msg.header.stamp = this->now();

//         pose_msg.pose.pose.position.x = x;
//         pose_msg.pose.pose.position.y = y;
//         pose_msg.pose.pose.position.z = 0.0;
//         pose_msg.pose.pose.orientation = yaw_to_quaternion(yaw);

//         std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
//         pose_msg.pose.covariance[0]  = 0.05;
//         pose_msg.pose.covariance[7]  = 0.05;
//         pose_msg.pose.covariance[35] = 0.02;

//         pose_pub_->publish(pose_msg);
//     }

//     // === Convert yaw to quaternion ===
//     geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
//     {
//         Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
//         geometry_msgs::msg::Quaternion q_msg;
//         q_msg.x = q.x();
//         q_msg.y = q.y();
//         q_msg.z = q.z();
//         q_msg.w = q.w();
//         return q_msg;
//     }

//     // === Publish visualization marker ===
//     void publish_marker(int tag_id, double x, double y)
//     {
//         visualization_msgs::msg::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = this->now();
//         marker.ns = "apriltag";
//         marker.id = tag_id;
//         marker.type = visualization_msgs::msg::Marker::SPHERE;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.pose.position.x = x;
//         marker.pose.position.y = y;
//         marker.pose.position.z = 0.1;
//         marker.pose.orientation.w = 1.0;
//         marker.scale.x = 0.2;
//         marker.scale.y = 0.2;
//         marker.scale.z = 0.05;
//         marker.color.r = 0.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;
//         marker.color.a = 0.7;
//         marker.lifetime = rclcpp::Duration::from_seconds(5.0);
//         marker_pub_->publish(marker);
//     }
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<AprilTagDriftRelocalizer>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <unordered_map>
#include <chrono>
#include <string>
#include <std_srvs/srv/trigger.hpp>  // <--- Service header

using namespace std::chrono_literals;

struct TagPose {
    double x, y, yaw;
};

class AprilTagDriftRelocalizer : public rclcpp::Node {
public:
    AprilTagDriftRelocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("apriltag_drift_relocalizer", options)
    {
        RCLCPP_INFO(this->get_logger(), "AprilTag drift relocalizer started.");

        // === Parameters ===
        this->declare_parameter<std::string>("tag_config", "config/tags.yaml");
        this->declare_parameter<double>("relocalize_check_period", 0.2);
        this->declare_parameter<double>("imu_yaw_offset", 0.0);
        this->declare_parameter<double>("search_angular_vel", 0.1);
        this->declare_parameter<double>("min_relocalize_distance", 1.5);  // NEW: minimum distance

        imu_yaw_offset_ = this->get_parameter("imu_yaw_offset").as_double();
        search_angular_vel_ = this->get_parameter("search_angular_vel").as_double();
        min_relocalize_distance_ = this->get_parameter("min_relocalize_distance").as_double();

        // === TF setup ===
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        camera_frame_ = "camera_link_optical";
        base_frame_ = "base_link";

        // === Publishers ===
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_tag", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/tag_markers", 10);

        // === IMU subscriber ===
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 20,
            std::bind(&AprilTagDriftRelocalizer::imu_callback, this, std::placeholders::_1)
        );

        // === Timer for drift check ===
        double check_period = this->get_parameter("relocalize_check_period").as_double();
        relocalize_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_period),
            std::bind(&AprilTagDriftRelocalizer::try_relocalize_from_tf, this)
        );

        // === Manual relocalization service ===
        relocalize_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_relocalization",
            std::bind(&AprilTagDriftRelocalizer::handle_relocalize_request, this,
                      std::placeholders::_1, std::placeholders::_2)
        );

        load_tag_config();
        localization_lost_ = true;  // start assuming drift
    }

private:
    // === Tag data ===
    std::unordered_map<int, TagPose> tag_map_;

    // === TF ===
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // === ROS pubs/subs ===
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr relocalize_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalize_service_;  // <--- service

    // === Frames & flags ===
    std::string camera_frame_;
    std::string base_frame_;
    bool localization_lost_ = true;
    bool relocalization_done_ = false;

    // === IMU data ===
    double latest_imu_yaw_ = 0.0;
    bool imu_received_ = false;
    double imu_yaw_offset_ = 0.0;

    // === Motion control ===
    double search_angular_vel_;
    rclcpp::Time last_rotate_cmd_time_;
    double min_relocalize_distance_;  // NEW

    // === Load tag world coordinates ===
    void load_tag_config()
    {
        std::string config_file = this->get_parameter("tag_config").as_string();
        std::string full_path = ament_index_cpp::get_package_share_directory("apriltag_relocalizer") + "/" + config_file;

        try {
            YAML::Node config = YAML::LoadFile(full_path);
            for (const auto& tag_node : config["tags"]) {
                int id = tag_node.first.as<int>();
                auto& t = tag_node.second;
                tag_map_[id] = {
                    t["x"].as<double>(),
                    t["y"].as<double>(),
                    t["yaw"].as<double>()
                };
                RCLCPP_INFO(this->get_logger(),
                    "Loaded tag %d: (x=%.3f, y=%.3f, yaw=%.3f)",
                    id, tag_map_[id].x, tag_map_[id].y, tag_map_[id].yaw);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load tag config: %s", e.what());
        }
    }

    // === IMU callback ===
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        latest_imu_yaw_ = yaw + imu_yaw_offset_;
        while (latest_imu_yaw_ > M_PI) latest_imu_yaw_ -= 2 * M_PI;
        while (latest_imu_yaw_ < -M_PI) latest_imu_yaw_ += 2 * M_PI;

        imu_received_ = true;
    }

    // === Relocalization logic ===
    void try_relocalize_from_tf()
    {
        if (!localization_lost_ || relocalization_done_) return;

        bool tag_found = false;

        for (const auto& [tag_id, tag_pose] : tag_map_) {
            std::string tag_frame = "tag36h11_" + std::to_string(tag_id);
            geometry_msgs::msg::TransformStamped tf_base_to_tag;

            try {
                tf_base_to_tag = tf_buffer_->lookupTransform(base_frame_, tag_frame, tf2::TimePointZero);
                tag_found = true;
            } catch (const tf2::TransformException & ex) {
                continue;
            }

            // === Compute distance from robot to tag ===
            double dx = tf_base_to_tag.transform.translation.x;
            double dy = tf_base_to_tag.transform.translation.y;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance > min_relocalize_distance_) {
                RCLCPP_INFO(this->get_logger(),
                    "Tag %d detected but too far (%.2f m > min %.2f m)",
                    tag_id, distance, min_relocalize_distance_);
                continue;  // skip this tag
            }

            // === Stop rotation ===
            stop_rotation();

            // === Extract base→tag translation ===
            Eigen::Vector3d t_base_to_tag(dx, dy, tf_base_to_tag.transform.translation.z);

            // === Compute map→base (robot) position ===
            double tag_x = tag_pose.x;
            double tag_y = tag_pose.y;
            double tag_yaw = tag_pose.yaw;
            Eigen::Rotation2Dd R_tag_to_map(tag_yaw);

            Eigen::Vector2d p_base_in_tag(t_base_to_tag.x(), t_base_to_tag.y());
            Eigen::Vector2d p_base_in_map = Eigen::Vector2d(tag_x, tag_y) - R_tag_to_map * p_base_in_tag;

            double world_x = p_base_in_map.x();
            double world_y = p_base_in_map.y();

            double world_yaw = imu_received_ ? latest_imu_yaw_ : tag_pose.yaw;
            while (world_yaw > M_PI) world_yaw -= 2 * M_PI;
            while (world_yaw < -M_PI) world_yaw += 2 * M_PI;

            RCLCPP_WARN(this->get_logger(),
                "RELOCALIZED using tag %d → Robot at (%.3f, %.3f, %.3f rad)",
                tag_id, world_x, world_y, world_yaw);

            publish_marker(tag_id, world_x, world_y);
            publish_initial_pose(world_x, world_y, world_yaw);

            relocalization_done_ = true;
            localization_lost_ = false;

            // === Exit the node after relocalization ===
            RCLCPP_INFO(this->get_logger(), "Relocalizer finished — shutting down node.");
            rclcpp::shutdown();  // <--- graceful exit
            return;
        }

        if (!tag_found) {
            rotate_to_search();
        }
    }

    // === Rotate slowly to search for tags ===
    void rotate_to_search()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = search_angular_vel_;
        cmd_pub_->publish(cmd);
    }

    // === Stop rotation ===
    void stop_rotation()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    // === Publish initial pose ===
    void publish_initial_pose(double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();

        pose_msg.pose.pose.position.x = x;
        pose_msg.pose.pose.position.y = y;
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation = yaw_to_quaternion(yaw);

        std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
        pose_msg.pose.covariance[0]  = 0.05;
        pose_msg.pose.covariance[7]  = 0.05;
        pose_msg.pose.covariance[35] = 0.02;

        pose_pub_->publish(pose_msg);
    }

    // === Convert yaw to quaternion ===
    geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
    {
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }

    // === Publish visualization marker ===
    void publish_marker(int tag_id, double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "apriltag";
        marker.id = tag_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.7;
        marker.lifetime = rclcpp::Duration::from_seconds(5.0);
        marker_pub_->publish(marker);
    }

    // === Service callback ===
    void handle_relocalize_request(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // unused
        RCLCPP_INFO(this->get_logger(), "Manual relocalization requested via service.");

        if (!localization_lost_) {
            response->success = false;
            response->message = "Localization is already valid, no need to relocalize.";
            return;
        }

        try_relocalize_from_tf();  // reuse existing logic

        if (relocalization_done_) {
            response->success = true;
            response->message = "Relocalization successful!";
        } else {
            response->success = false;
            response->message = "Relocalization failed: no tag detected nearby.";
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDriftRelocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

