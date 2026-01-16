// Copyright 2025 OTAGG Team
// Licensed under Apache 2.0

#ifndef NAVIGATION_2025__BT_CUSTOM_NODES_HPP_
#define NAVIGATION_2025__BT_CUSTOM_NODES_HPP_

#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>

namespace navigation_2025 {

/**
 * @brief Detects if robot is oscillating (high variance, low net displacement)
 */
class DetectOscillation : public BT::ConditionNode {
public:
  DetectOscillation(const std::string &name,
                    const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    getInput("threshold", threshold_);
    getInput("time_window", time_window_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          geometry_msgs::msg::PoseStamped pose;
          pose.header = msg->header;
          pose.pose = msg->pose.pose;
          pose_history_.push_back(pose);

          // Keep only poses within time window
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

    // Calculate position variance
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

    // Calculate net displacement
    double net_dx = pose_history_.back().pose.position.x -
                    pose_history_.front().pose.position.x;
    double net_dy = pose_history_.back().pose.position.y -
                    pose_history_.front().pose.position.y;
    double net_displacement = std::sqrt(net_dx * net_dx + net_dy * net_dy);

    // High variance + low net displacement = oscillation
    if (variance > threshold_ && net_displacement < 0.5) {
      RCLCPP_WARN(node_->get_logger(),
                  "Oscillation detected! variance=%.2f, displacement=%.2f",
                  variance, net_displacement);
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

/**
 * @brief Checks if robot has diverged too far from planned path
 */
class PathDivergenceCheck : public BT::ConditionNode {
public:
  PathDivergenceCheck(const std::string &name,
                      const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    getInput("max_divergence", max_divergence_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
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

    // Find closest point on path
    double min_distance = std::numeric_limits<double>::max();
    for (const auto &pose : path.poses) {
      double dx = current_pose_.position.x - pose.pose.position.x;
      double dy = current_pose_.position.y - pose.pose.position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      min_distance = std::min(min_distance, dist);
    }

    if (min_distance > max_divergence_) {
      RCLCPP_WARN(node_->get_logger(), "Path divergence: %.2fm", min_distance);
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

/**
 * @brief Checks if velocity is too low for too long (stuck detection)
 */
class LowVelocityCheck : public BT::ConditionNode {
public:
  LowVelocityCheck(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    getInput("min_velocity", min_velocity_);
    getInput("duration", duration_threshold_);

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          current_vel_ = msg->twist.twist;
          last_update_ = node_->now();
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
        RCLCPP_WARN(node_->get_logger(), "Low velocity for %.1fs", duration);
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
  rclcpp::Time last_update_;
  rclcpp::Time low_vel_start_time_{0};
  std::mutex mutex_;
  double min_velocity_{0.1};
  double duration_threshold_{5.0};
};

/**
 * @brief Checks if it's safe to backup by checking rear clearance
 */
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

    // Check rear clearance (angles around π radians)
    double min_rear_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < latest_scan_.ranges.size(); i++) {
      double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

      // Rear 90-degree sector (π ± 45°)
      if (std::abs(angle - M_PI) < M_PI / 4 ||
          std::abs(angle + M_PI) < M_PI / 4) {
        if (latest_scan_.ranges[i] > latest_scan_.range_min &&
            latest_scan_.ranges[i] < latest_scan_.range_max) {
          min_rear_distance = std::min(
              min_rear_distance, static_cast<double>(latest_scan_.ranges[i]));
        }
      }
    }

    double required_clearance = required_distance_ + 0.5; // Add safety margin

    if (min_rear_distance > required_clearance) {
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(node_->get_logger(),
                "Unsafe to backup: rear clearance %.2fm < required %.2fm",
                min_rear_distance, required_clearance);
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

/**
 * @brief Checks if there's enough 360° clearance for complex maneuvers
 */
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

    RCLCPP_WARN(node_->get_logger(), "Insufficient clearance for maneuver");
    return BT::NodeStatus::FAILURE;
  }

private:
  bool checkSectorClearance(double center_angle, double half_width,
                            double required_distance) {
    double min_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < latest_scan_.ranges.size(); i++) {
      double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;

      // Normalize angle difference
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
// Traffic-Aware Condition Nodes
// ============================================================================

// Forward declare the TrafficState message
} // namespace navigation_2025

#include <otagg_vision_interfaces/msg/traffic_state.hpp>

namespace navigation_2025 {

/**
 * @brief Base class for traffic state condition nodes
 */
class TrafficStateCondition : public BT::ConditionNode {
protected:
  TrafficStateCondition(const std::string &name,
                        const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    state_sub_ = node_->create_subscription<otagg_vision_interfaces::msg::TrafficState>(
        "/traffic_state", 10,
        [this](const otagg_vision_interfaces::msg::TrafficState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_state_ = *msg;
          has_state_ = true;
        });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<otagg_vision_interfaces::msg::TrafficState>::SharedPtr state_sub_;
  otagg_vision_interfaces::msg::TrafficState latest_state_;
  std::mutex mutex_;
  bool has_state_{false};
};

/**
 * @brief Returns SUCCESS when red light detected within distance threshold
 */
class IsRedLightDetected : public TrafficStateCondition {
public:
  IsRedLightDetected(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {
    getInput("distance_threshold", distance_threshold_);
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("distance_threshold", 20.0, "Max distance to consider (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    // Check if red light and within distance
    if (latest_state_.traffic_light_state == 3 &&  // RED
        latest_state_.traffic_light_distance > 0 &&
        latest_state_.traffic_light_distance < distance_threshold_) {
      RCLCPP_INFO(node_->get_logger(), "Red light detected at %.1fm",
                  latest_state_.traffic_light_distance);
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  double distance_threshold_{20.0};
};

/**
 * @brief Returns SUCCESS when yellow light detected within distance threshold
 */
class IsYellowLightDetected : public TrafficStateCondition {
public:
  IsYellowLightDetected(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {
    getInput("distance_threshold", distance_threshold_);
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("distance_threshold", 20.0, "Max distance to consider (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_state_.traffic_light_state == 2 &&  // YELLOW
        latest_state_.traffic_light_distance > 0 &&
        latest_state_.traffic_light_distance < distance_threshold_) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  double distance_threshold_{20.0};
};

/**
 * @brief Returns SUCCESS when stop sign detected within distance threshold
 */
class IsStopSignDetected : public TrafficStateCondition {
public:
  IsStopSignDetected(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {
    getInput("distance_threshold", distance_threshold_);
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("distance_threshold", 10.0, "Max distance to consider (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_state_.stop_sign_detected &&
        latest_state_.stop_sign_distance > 0 &&
        latest_state_.stop_sign_distance < distance_threshold_) {
      RCLCPP_INFO(node_->get_logger(), "Stop sign detected at %.1fm",
                  latest_state_.stop_sign_distance);
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  double distance_threshold_{10.0};
};

/**
 * @brief Returns SUCCESS when specified turn is restricted (no_left_turn or no_right_turn)
 */
class IsTurnRestricted : public TrafficStateCondition {
public:
  IsTurnRestricted(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {
    getInput("direction", direction_);
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("direction", "left", "Turn direction to check: 'left' or 'right'")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (direction_ == "left" && latest_state_.no_left_turn) {
      return BT::NodeStatus::SUCCESS;
    }
    if (direction_ == "right" && latest_state_.no_right_turn) {
      return BT::NodeStatus::SUCCESS;
    }
    if (latest_state_.no_entry || latest_state_.road_closed) {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  std::string direction_{"left"};
};

/**
 * @brief Returns SUCCESS when bus stop detected within distance threshold
 */
class IsBusStopDetected : public TrafficStateCondition {
public:
  IsBusStopDetected(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {
    getInput("distance_threshold", distance_threshold_);
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<double>("distance_threshold", 15.0, "Max distance to consider (m)")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_state_.bus_stop_detected &&
        latest_state_.bus_stop_distance > 0 &&
        latest_state_.bus_stop_distance < distance_threshold_) {
      RCLCPP_INFO(node_->get_logger(), "Bus stop detected at %.1fm",
                  latest_state_.bus_stop_distance);
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

private:
  double distance_threshold_{15.0};
};

/**
 * @brief Returns SUCCESS when a speed limit is active (non-zero)
 */
class IsSpeedLimitActive : public TrafficStateCondition {
public:
  IsSpeedLimitActive(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::OutputPort<uint8_t>("speed_limit", "Detected speed limit in km/h")};
  }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_state_.speed_limit_kmh > 0) {
      setOutput("speed_limit", latest_state_.speed_limit_kmh);
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

/**
 * @brief Returns SUCCESS when traffic light is green (for WaitForGreenLight use)
 */
class IsGreenLightDetected : public TrafficStateCondition {
public:
  IsGreenLightDetected(const std::string &name, const BT::NodeConfiguration &config)
      : TrafficStateCondition(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_state_) {
      return BT::NodeStatus::FAILURE;
    }

    if (latest_state_.traffic_light_state == 1) {  // GREEN
      RCLCPP_INFO(node_->get_logger(), "Green light detected!");
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }
};

} // namespace navigation_2025

#endif // NAVIGATION_2025__BT_CUSTOM_NODES_HPP_

