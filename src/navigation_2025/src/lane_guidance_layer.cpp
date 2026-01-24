/**
 * @file lane_guidance_layer.cpp
 * @brief Implementation of the Lane Guidance Layer for Nav2 costmaps
 *
 * This implementation provides soft lane guidance by marking lane detection
 * points with configurable medium costs rather than treating them as obstacles.
 *
 * @author OTAGG Team
 * @date 2026
 */

#include "navigation_2025/lane_guidance_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(navigation_2025::LaneGuidanceLayer, nav2_costmap_2d::Layer)

namespace navigation_2025
{

LaneGuidanceLayer::LaneGuidanceLayer()
: last_min_x_(0.0),
  last_min_y_(0.0),
  last_max_x_(0.0),
  last_max_y_(0.0),
  bounds_initialized_(false),
  points_received_count_(0),
  points_applied_count_(0)
{
}

LaneGuidanceLayer::~LaneGuidanceLayer()
{
  if (cloud_sub_) {
    cloud_sub_.reset();
  }
}

void LaneGuidanceLayer::onInitialize()
{
  node_ = node_.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock node in LaneGuidanceLayer");
  }

  // Declare and get parameters with defaults
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue("/lane_detection/pointcloud"));
  declareParameter("lane_cost", rclcpp::ParameterValue(100));
  declareParameter("min_height", rclcpp::ParameterValue(-1.0));
  declareParameter("max_height", rclcpp::ParameterValue(1.0));
  declareParameter("max_point_age", rclcpp::ParameterValue(2.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  declareParameter("use_temporal_decay", rclcpp::ParameterValue(true));

  node_->get_parameter(name_ + "." + "enabled", enabled_);
  node_->get_parameter(name_ + "." + "pointcloud_topic", pointcloud_topic_);

  int lane_cost_int;
  node_->get_parameter(name_ + "." + "lane_cost", lane_cost_int);
  lane_cost_ = static_cast<unsigned char>(std::clamp(lane_cost_int, 0, 254));

  node_->get_parameter(name_ + "." + "min_height", min_height_);
  node_->get_parameter(name_ + "." + "max_height", max_height_);
  node_->get_parameter(name_ + "." + "max_point_age", max_point_age_);
  node_->get_parameter(name_ + "." + "transform_tolerance", transform_tolerance_);
  node_->get_parameter(name_ + "." + "use_temporal_decay", use_temporal_decay_);

  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscription to lane detection point cloud
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&LaneGuidanceLayer::pointCloudCallback, this, std::placeholders::_1)
  );

  current_ = true;  // Mark layer as current

  RCLCPP_INFO(
    node_->get_logger(),
    "LaneGuidanceLayer initialized: topic='%s', cost=%d, height_range=[%.2f, %.2f]",
    pointcloud_topic_.c_str(), lane_cost_, min_height_, max_height_
  );
}

void LaneGuidanceLayer::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating LaneGuidanceLayer");
  enabled_ = true;
}

void LaneGuidanceLayer::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating LaneGuidanceLayer");
  enabled_ = false;
  std::lock_guard<std::mutex> lock(points_mutex_);
  lane_points_.clear();
}

void LaneGuidanceLayer::reset()
{
  RCLCPP_INFO(node_->get_logger(), "Resetting LaneGuidanceLayer");
  std::lock_guard<std::mutex> lock(points_mutex_);
  lane_points_.clear();
  bounds_initialized_ = false;
  resetMaps();
}

void LaneGuidanceLayer::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  if (!enabled_) {
    return;
  }

  points_received_count_++;
  processPointCloud(cloud);
}

void LaneGuidanceLayer::processPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  // Transform point cloud to global frame
  sensor_msgs::msg::PointCloud2 transformed_cloud;

  try {
    // Get the transform from cloud frame to global frame
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(),
      cloud->header.frame_id,
      cloud->header.stamp,
      tf2::durationFromSec(transform_tolerance_)
    );

    // Transform the point cloud
    tf2::doTransform(*cloud, transformed_cloud, transform);

  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,  // 5 seconds throttle
      "Could not transform point cloud from %s to %s: %s",
      cloud->header.frame_id.c_str(),
      layered_costmap_->getGlobalFrameID().c_str(),
      ex.what()
    );
    return;
  }

  // Parse and store points
  std::lock_guard<std::mutex> lock(points_mutex_);

  // Iterate through point cloud
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");

  rclcpp::Time current_time = node_->now();
  size_t points_added = 0;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Filter by height
    if (z < min_height_ || z > max_height_) {
      continue;
    }

    // Check if point is within costmap bounds
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }

    // Add point to storage
    LanePoint point;
    point.x = static_cast<double>(x);
    point.y = static_cast<double>(y);
    point.timestamp = current_time;

    lane_points_.push_back(point);
    points_added++;
  }

  // Decay old points if temporal decay is enabled
  if (use_temporal_decay_) {
    decayLanePoints();
  }

  if (points_added > 0) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Processed %zu lane points, total stored: %zu",
      points_added, lane_points_.size()
    );
  }
}

void LaneGuidanceLayer::decayLanePoints()
{
  // Remove points older than max_point_age_
  rclcpp::Time current_time = node_->now();
  rclcpp::Duration max_age = rclcpp::Duration::from_seconds(max_point_age_);

  auto it = lane_points_.begin();
  while (it != lane_points_.end()) {
    if ((current_time - it->timestamp) > max_age) {
      it = lane_points_.erase(it);
    } else {
      ++it;
    }
  }
}

void LaneGuidanceLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(points_mutex_);

  if (lane_points_.empty()) {
    return;
  }

  // Initialize bounds with robot position if first update
  if (!bounds_initialized_) {
    last_min_x_ = robot_x;
    last_min_y_ = robot_y;
    last_max_x_ = robot_x;
    last_max_y_ = robot_y;
    bounds_initialized_ = true;
  }

  // Update bounds based on lane points
  double local_min_x = std::numeric_limits<double>::max();
  double local_min_y = std::numeric_limits<double>::max();
  double local_max_x = std::numeric_limits<double>::lowest();
  double local_max_y = std::numeric_limits<double>::lowest();

  bool found_valid_point = false;

  for (const auto & point : lane_points_) {
    local_min_x = std::min(local_min_x, point.x);
    local_min_y = std::min(local_min_y, point.y);
    local_max_x = std::max(local_max_x, point.x);
    local_max_y = std::max(local_max_y, point.y);
    found_valid_point = true;
  }

  if (found_valid_point) {
    // Add small buffer to bounds
    const double buffer = 0.5;  // 0.5 meter buffer
    local_min_x -= buffer;
    local_min_y -= buffer;
    local_max_x += buffer;
    local_max_y += buffer;

    // Update the output bounds
    *min_x = std::min(*min_x, local_min_x);
    *min_y = std::min(*min_y, local_min_y);
    *max_x = std::max(*max_x, local_max_x);
    *max_y = std::max(*max_y, local_max_y);

    // Store for next iteration
    last_min_x_ = local_min_x;
    last_min_y_ = local_min_y;
    last_max_x_ = local_max_x;
    last_max_y_ = local_max_y;
  }
}

void LaneGuidanceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(points_mutex_);

  if (lane_points_.empty()) {
    return;
  }

  last_update_time_ = node_->now();
  points_applied_count_ = 0;

  // Get costmap properties
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  // Iterate through all stored lane points
  for (const auto & point : lane_points_) {
    // Convert world coordinates to costmap coordinates
    unsigned int mx, my;
    if (!master_grid.worldToMap(point.x, point.y, mx, my)) {
      continue;  // Point is outside costmap bounds
    }

    // Check if within update bounds
    if (static_cast<int>(mx) < min_i || static_cast<int>(mx) >= max_i ||
        static_cast<int>(my) < min_j || static_cast<int>(my) >= max_j)
    {
      continue;
    }

    // Set the cost at this cell
    // Use std::max to ensure we don't overwrite higher costs (like obstacles)
    // but we can upgrade free space to lane guidance
    unsigned char current_cost = master_grid.getCost(mx, my);

    // Only update if current cost is lower (free space or lower guidance)
    // This prevents overwriting actual obstacles with lane guidance
    if (current_cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      master_grid.setCost(mx, my, std::max(current_cost, lane_cost_));
      points_applied_count_++;
    }
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "Updated costmap with %zu lane points (applied: %zu)",
    lane_points_.size(), points_applied_count_
  );
}

}  // namespace navigation_2025
