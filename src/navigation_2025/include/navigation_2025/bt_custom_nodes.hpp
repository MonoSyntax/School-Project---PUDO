// Copyright 2025 OTAGG Team
// Licensed under Apache 2.0

#ifndef NAVIGATION_2025__BT_CUSTOM_NODES_HPP_
#define NAVIGATION_2025__BT_CUSTOM_NODES_HPP_

#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <otagg_vision_interfaces/msg/traffic_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace navigation_2025 {

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Calculate 2D distance between two points
 */
inline double distance2D(double x1, double y1, double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Normalize angle to [-pi, pi]
 */
inline double normalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

/**
 * @brief Calculate heading between two points
 */
inline double calculateHeading(double x1, double y1, double x2, double y2) {
  return std::atan2(y2 - y1, x2 - x1);
}

/**
 * @brief Find closest point on path to a given position
 */
inline size_t findClosestPathPoint(const nav_msgs::msg::Path &path, double x,
                                   double y) {

  if (path.poses.empty())
    return 0;

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); i++) {
    double dist = distance2D(x, y, path.poses[i].pose.position.x,
                             path.poses[i].pose.position.y);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

/**
 * @brief Check if a point is within distance threshold of path
 */
inline bool isPointNearPath(const nav_msgs::msg::Path &path, double point_x,
                            double point_y, double threshold,
                            size_t lookahead_points = 20) {

  if (path.poses.empty())
    return false;

  // Find closest point on path
  size_t start_idx = findClosestPathPoint(path, point_x, point_y);

  // Check points ahead on path (within lookahead)
  size_t end_idx = std::min(start_idx + lookahead_points, path.poses.size());

  for (size_t i = start_idx; i < end_idx; i++) {
    double dist = distance2D(point_x, point_y, path.poses[i].pose.position.x,
                             path.poses[i].pose.position.y);

    if (dist < threshold) {
      return true;
    }
  }

  return false;
}

// ============================================================================
// RECOVERY CONDITION NODES
// ============================================================================

class DetectOscillation : public BT::ConditionNode {
public:
  DetectOscillation(const std::string &name,
                    const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("threshold", threshold_);
    getInput("time_window", time_window_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          geometry_msgs::msg::PoseStamped pose;
          pose.header = msg->header;
          pose.pose = msg->pose.pose;
          pose_history_.push_back(pose);

          auto now = node_->now();
          while (!pose_history_.empty()) {
            auto age = (now - pose_history_.front().header.stamp).seconds();
            if (age > time_window_) {
              pose_history_.pop_front();
            } else {
              break;
            }
          }
        });
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("threshold", 0.3, "Oscillation threshold"),
        BT::InputPort<double>("time_window", 10.0, "Time window in seconds")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pose_history_.size() < 5) {
      return BT::NodeStatus::FAILURE;
    }

    double sum_x = 0, sum_y = 0;
    for (const auto &pose : pose_history_) {
      sum_x += pose.pose.position.x;
      sum_y += pose.pose.position.y;
    }
    double mean_x = sum_x / pose_history_.size();
    double mean_y = sum_y / pose_history_.size();

    double variance = 0;
    for (const auto &pose : pose_history_) {
      double dx = pose.pose.position.x - mean_x;
      double dy = pose.pose.position.y - mean_y;
      variance += dx * dx + dy * dy;
    }
    variance /= pose_history_.size();

    double net_dx = pose_history_.back().pose.position.x -
                    pose_history_.front().pose.position.x;
    double net_dy = pose_history_.back().pose.position.y -
                    pose_history_.front().pose.position.y;
    double net_displacement = std::sqrt(net_dx * net_dx + net_dy * net_dy);

    if (variance > threshold_ && net_displacement < 0.5) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_history_;
  std::mutex mutex_;
  double threshold_{0.3};
  double time_window_{10.0};
};

class PathDivergenceCheck : public BT::ConditionNode {
public:
  PathDivergenceCheck(const std::string &name,
                      const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("max_divergence", max_divergence_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          current_pose_ = msg->pose.pose;
          has_pose_ = true;
        });
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("max_divergence", 2.0, "Max path divergence (m)"),
        BT::InputPort<nav_msgs::msg::Path>("path", "Current path")};
  }

  BT::NodeStatus tick() override {
    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
      return BT::NodeStatus::FAILURE;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_pose_ || path.poses.empty()) {
      return BT::NodeStatus::FAILURE;
    }

    double min_distance = std::numeric_limits<double>::max();
    for (const auto &pose : path.poses) {
      double dx = current_pose_.position.x - pose.pose.position.x;
      double dy = current_pose_.position.y - pose.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      min_distance = std::min(min_distance, dist);
    }

    if (min_distance > max_divergence_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose current_pose_;
  std::mutex mutex_;
  bool has_pose_{false};
  double max_divergence_{2.0};
};

class LowVelocityCheck : public BT::ConditionNode {
public:
  LowVelocityCheck(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("min_velocity", min_velocity_);
    getInput("duration", duration_threshold_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          current_vel_ = msg->twist.twist;
        });
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("min_velocity", 0.1,
                                  "Min velocity threshold (m/s)"),
            BT::InputPort<double>("duration", 5.0, "Duration threshold (s)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    double speed = std::sqrt(current_vel_.linear.x * current_vel_.linear.x +
                             current_vel_.linear.y * current_vel_.linear.y);
    auto now = node_->now();

    if (speed < min_velocity_) {
      if (low_vel_start_time_.nanoseconds() == 0) {
        low_vel_start_time_ = now;
      }
      double duration = (now - low_vel_start_time_).seconds();
      if (duration > duration_threshold_) {
        return BT::NodeStatus::SUCCESS;
      }
    } else {
      low_vel_start_time_ = rclcpp::Time(0);
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Twist current_vel_;
  rclcpp::Time low_vel_start_time_{0};
  std::mutex mutex_;
  double min_velocity_{0.1};
  double duration_threshold_{5.0};
};

class IsSafeToBackup : public BT::ConditionNode {
public:
  IsSafeToBackup(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("distance", required_distance_);

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_scan_ = *msg;
          has_scan_ = true;
        });
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("distance", 1.0, "Required backup distance (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_scan_) {
      return BT::NodeStatus::FAILURE;
    }

    double min_rear_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < latest_scan_.ranges.size(); i++) {
      double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
      if (std::abs(angle - M_PI) < M_PI / 4 ||
          std::abs(angle + M_PI) < M_PI / 4) {
        if (latest_scan_.ranges[i] > latest_scan_.range_min &&
            latest_scan_.ranges[i] < latest_scan_.range_max) {
          min_rear_distance = std::min(
              min_rear_distance, static_cast<double>(latest_scan_.ranges[i]));
        }
      }
    }

    double required_clearance = required_distance_ + 0.5;
    if (min_rear_distance > required_clearance) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan latest_scan_;
  std::mutex mutex_;
  bool has_scan_{false};
  double required_distance_{1.0};
};

class HasClearanceForManeuver : public BT::ConditionNode {
public:
  HasClearanceForManeuver(const std::string &name,
                          const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("front_clearance", front_clearance_);
    getInput("side_clearance", side_clearance_);
    getInput("rear_clearance", rear_clearance_);

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_scan_ = *msg;
          has_scan_ = true;
        });
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("front_clearance", 3.0, "Front clearance (m)"),
        BT::InputPort<double>("side_clearance", 2.0, "Side clearance (m)"),
        BT::InputPort<double>("rear_clearance", 2.0, "Rear clearance (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_scan_) {
      return BT::NodeStatus::FAILURE;
    }

    bool front_clear = checkSectorClearance(0.0, M_PI / 4, front_clearance_);
    bool left_clear = checkSectorClearance(M_PI / 2, M_PI / 4, side_clearance_);
    bool right_clear =
        checkSectorClearance(-M_PI / 2, M_PI / 4, side_clearance_);
    bool rear_clear = checkSectorClearance(M_PI, M_PI / 4, rear_clearance_);

    if (front_clear && left_clear && right_clear && rear_clear) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  bool checkSectorClearance(double center_angle, double half_width,
                            double required_distance) {
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < latest_scan_.ranges.size(); i++) {
      double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
      double diff = angle - center_angle;
      while (diff > M_PI)
        diff -= 2 * M_PI;
      while (diff < -M_PI)
        diff += 2 * M_PI;
      if (std::abs(diff) < half_width) {
        if (latest_scan_.ranges[i] > latest_scan_.range_min &&
            latest_scan_.ranges[i] < latest_scan_.range_max) {
          min_dist =
              std::min(min_dist, static_cast<double>(latest_scan_.ranges[i]));
        }
      }
    }
    return min_dist > required_distance;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan latest_scan_;
  std::mutex mutex_;
  bool has_scan_{false};
  double front_clearance_{3.0};
  double side_clearance_{2.0};
  double rear_clearance_{2.0};
};

// ============================================================================
// TRAFFIC DETECTION NODES (DISTANCE-BASED WITH NAV OVERRIDE)
// ============================================================================

/**
 * @brief Base class for traffic detection using distance + nav override
 *
 * Since TrafficState only provides distance (not world coordinates),
 * we use a simpler approach:
 * 1. Check distance is in valid zone [min, max]
 * 2. Check nav_override_state if available
 * 3. Trust that vision system only reports signs ahead in view
 */
class TrafficConditionBase : public BT::ConditionNode {
protected:
  TrafficConditionBase(const std::string &name,
                       const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Subscribe to traffic state
    state_sub_ =
        node_->create_subscription<otagg_vision_interfaces::msg::TrafficState>(
            "/traffic_state", 10,
            [this](const otagg_vision_interfaces::msg::TrafficState::SharedPtr
                       msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              latest_state_ = *msg;
              has_state_ = true;
              last_update_time_ = node_->now();
            });
  }

  /**
   * @brief Check if detection is valid (in distance zone and fresh)
   */
  bool isValidDetection(float distance, double min_dist, double max_dist) {
    // Check if data is recent (within 1 second)
    auto now = node_->now();
    auto age = (now - last_update_time_).seconds();
    if (age > 1.0) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Traffic state too old: %.2fs", age);
      return false;
    }

    // Check distance is valid
    // Check distance is valid
    if (distance <= 0.0f) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Invalid distance: %.2f", distance);
      return false;
    }

    // Check distance zone
    if (distance < min_dist) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Distance %.2fm < min %.2fm (already passed)",
                            distance, min_dist);
      return false;
    }

    if (distance > max_dist) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                            "Distance %.2fm > max %.2fm (too far)", distance,
                            max_dist);
      return false;
    }

    return true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<otagg_vision_interfaces::msg::TrafficState>::SharedPtr
      state_sub_;
  otagg_vision_interfaces::msg::TrafficState latest_state_;
  rclcpp::Time last_update_time_;
  std::mutex mutex_;
  bool has_state_{false};
};

/**
 * @brief Checks if red traffic light should trigger stop
 */
class IsRedLightDetected : public TrafficConditionBase {
public:
  IsRedLightDetected(const std::string &name,
                     const BT::NodeConfiguration &config)
      : TrafficConditionBase(name, config) {
    getInput("min_distance", min_distance_);
    getInput("max_distance", max_distance_);
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("min_distance", 3.0,
                                  "Min detection distance (m)"),
            BT::InputPort<double>("max_distance", 15.0,
                                  "Max detection distance (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if red light is detected
    if (latest_state_.traffic_light_state != latest_state_.LIGHT_RED) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if detection is valid (distance and freshness)
    if (!isValidDetection(latest_state_.traffic_light_distance, min_distance_,
                          max_distance_)) {
      return BT::NodeStatus::FAILURE;
    }

    // Check nav override if present
    // Check nav override if present
    if (latest_state_.nav_override_state == latest_state_.NAV_OVERRIDE_STOP) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "🚦 RED LIGHT - STOPPING (dist: %.2fm, override: STOP)",
          latest_state_.traffic_light_distance);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "🚦 RED LIGHT - STOPPING (dist: %.2fm)",
                         latest_state_.traffic_light_distance);
    return BT::NodeStatus::SUCCESS;
  }

private:
  double min_distance_{3.0};
  double max_distance_{15.0};
};

/**
 * @brief Checks if yellow traffic light should trigger stop
 */
class IsYellowLightDetected : public TrafficConditionBase {
public:
  IsYellowLightDetected(const std::string &name,
                        const BT::NodeConfiguration &config)
      : TrafficConditionBase(name, config) {
    getInput("min_distance", min_distance_);
    getInput("max_distance", max_distance_);
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("min_distance", 3.0,
                                  "Min detection distance (m)"),
            BT::InputPort<double>("max_distance", 15.0,
                                  "Max detection distance (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if yellow light is detected
    if (latest_state_.traffic_light_state != latest_state_.LIGHT_YELLOW) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if detection is valid
    if (!isValidDetection(latest_state_.traffic_light_distance, min_distance_,
                          max_distance_)) {
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "🟡 YELLOW LIGHT - STOPPING (dist: %.2fm)",
                         latest_state_.traffic_light_distance);
    return BT::NodeStatus::SUCCESS;
  }

private:
  double min_distance_{3.0};
  double max_distance_{15.0};
};

/**
 * @brief Checks if green light is detected (allows proceeding)
 */
class IsGreenLightDetected : public TrafficConditionBase {
public:
  IsGreenLightDetected(const std::string &name,
                       const BT::NodeConfiguration &config)
      : TrafficConditionBase(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_state_.traffic_light_state == latest_state_.LIGHT_GREEN) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "🟢 GREEN LIGHT - PROCEEDING");
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

/**
 * @brief Checks if stop sign should trigger stop
 */
class IsStopSignDetected : public TrafficConditionBase {
public:
  IsStopSignDetected(const std::string &name,
                     const BT::NodeConfiguration &config)
      : TrafficConditionBase(name, config) {
    getInput("min_distance", min_distance_);
    getInput("max_distance", max_distance_);
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("min_distance", 3.0,
                                  "Min detection distance (m)"),
            BT::InputPort<double>("max_distance", 15.0,
                                  "Max detection distance (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_ || !latest_state_.stop_sign_detected) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if detection is valid
    if (!isValidDetection(latest_state_.stop_sign_distance, min_distance_,
                          max_distance_)) {
      return BT::NodeStatus::FAILURE;
    }

    // Check nav override if present
    // Check nav override if present
    if (latest_state_.nav_override_state == latest_state_.NAV_OVERRIDE_STOP) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "🛑 STOP SIGN - STOPPING (dist: %.2fm, override: STOP)",
          latest_state_.stop_sign_distance);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "🛑 STOP SIGN - STOPPING (dist: %.2fm)",
                         latest_state_.stop_sign_distance);
    return BT::NodeStatus::SUCCESS;
  }

private:
  double min_distance_{3.0};
  double max_distance_{15.0};
};

// ============================================================================
// ACTION NODES
// ============================================================================

/**
 * @brief Publishes zero velocity to stop the robot
 */
class PublishZeroVelocity : public BT::SyncActionNode {
public:
  PublishZeroVelocity(const std::string &name,
                      const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("duration", duration_);

    // Publisher for cmd_vel
    cmd_vel_pub_ =
        node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>(
        "duration", 0.5, "How long to publish zero velocity (seconds)")};
  }

  BT::NodeStatus tick() override {
    geometry_msgs::msg::Twist zero_vel;
    zero_vel.linear.x = 0.0;
    zero_vel.linear.y = 0.0;
    zero_vel.linear.z = 0.0;
    zero_vel.angular.x = 0.0;
    zero_vel.angular.y = 0.0;
    zero_vel.angular.z = 0.0;

    auto start = node_->now();
    auto duration_ns =
        std::chrono::nanoseconds(static_cast<int64_t>(duration_ * 1e9));
    rclcpp::Duration duration(duration_ns);

    while ((node_->now() - start) < duration) {
      cmd_vel_pub_->publish(zero_vel);
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Robot stopped (zero velocity published)");
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  double duration_{0.5};
};

} // namespace navigation_2025

#endif // NAVIGATION_2025__BT_CUSTOM_NODES_HPP_