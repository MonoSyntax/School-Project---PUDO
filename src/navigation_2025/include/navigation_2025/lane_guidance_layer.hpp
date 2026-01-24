/**
 * @file lane_guidance_layer.hpp
 * @brief Custom Nav2 costmap layer for lane detection guidance
 *
 * This layer treats lane markings as soft guidance rather than hard obstacles.
 * It assigns configurable medium costs to encourage lane-following behavior
 * while still allowing the planner to cross lanes for turns and lane changes.
 *
 * @author OTAGG Team
 * @date 2026
 */

#ifndef NAVIGATION_2025__LANE_GUIDANCE_LAYER_HPP_
#define NAVIGATION_2025__LANE_GUIDANCE_LAYER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace navigation_2025 {

/**
 * @class LaneGuidanceLayer
 * @brief A costmap layer that provides soft guidance based on lane detection
 *
 * This layer subscribes to lane detection point clouds and marks them with
 * a configurable medium cost value. Unlike obstacle layers that mark lanes
 * as lethal (impassable), this layer encourages staying within lanes while
 * still allowing path planning across lane markings when necessary.
 */
class LaneGuidanceLayer : public nav2_costmap_2d::Layer {
public:
  /**
   * @brief Constructor
   */
  LaneGuidanceLayer();

  /**
   * @brief Destructor
   */
  virtual ~LaneGuidanceLayer();

  /**
   * @brief Initialization method called after construction
   *
   * Sets up ROS parameters, creates subscribers, and initializes TF buffer.
   */
  virtual void onInitialize() override;

  /**
   * @brief Update the bounds of the costmap that need to be updated
   *
   * @param robot_x Current robot x position
   * @param robot_y Current robot y position
   * @param robot_yaw Current robot yaw
   * @param min_x Minimum x bound to update
   * @param min_y Minimum y bound to update
   * @param max_x Maximum x bound to update
   * @param max_y Maximum y bound to update
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double *min_x, double *min_y, double *max_x,
                            double *max_y) override;

  /**
   * @brief Update the costs in the costmap based on lane detection
   *
   * @param master_grid The master costmap to update
   * @param min_x Minimum x bound to update
   * @param min_y Minimum y bound to update
   * @param max_x Maximum x bound to update
   * @param max_y Maximum y bound to update
   */
  virtual void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                           int min_j, int max_i, int max_j) override;

  /**
   * @brief Reset the layer
   */
  virtual void reset() override;

  /**
   * @brief Activate the layer
   */
  virtual void activate() override;

  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate() override;

  /**
   * @brief Check if the layer is using size matching
   * @return Always returns false as this layer doesn't require size matching
   */
  virtual bool isClearable() override { return false; }

private:
  /**
   * @brief Callback for receiving lane detection point clouds
   * @param cloud The incoming point cloud message
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  /**
   * @brief Transform and process point cloud data
   * @param cloud The point cloud to process
   */
  void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  /**
   * @brief Clear old lane points based on decay time
   */
  void decayLanePoints();

  // ROS interfaces

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string pointcloud_topic_; ///< Topic to subscribe for lane point clouds
  unsigned char lane_cost_;      ///< Cost value to assign to lane cells (0-255)
  double min_height_;            ///< Minimum point height to consider
  double max_height_;            ///< Maximum point height to consider
  double max_point_age_;       ///< Maximum age of points before decay (seconds)
  double transform_tolerance_; ///< TF transform tolerance
  bool use_temporal_decay_;    ///< Enable time-based decay of lane points

  // Data structures for point storage
  struct LanePoint {
    double x;               ///< World X coordinate
    double y;               ///< World Y coordinate
    rclcpp::Time timestamp; ///< When this point was received
  };

  std::deque<LanePoint> lane_points_; ///< Stored lane points with timestamps
  std::mutex points_mutex_; ///< Mutex for thread-safe access to lane_points_

  // Update bounds tracking
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool bounds_initialized_;

  // Performance tracking
  rclcpp::Time last_update_time_;
  size_t points_received_count_;
  size_t points_applied_count_;
};

} // namespace navigation_2025

#endif // NAVIGATION_2025__LANE_GUIDANCE_LAYER_HPP_
