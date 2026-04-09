#ifndef NAV2_MPPI_CONTROLLER__CRITICS__ADAPTIVE_HEADING_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__ADAPTIVE_HEADING_CRITIC_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mppi::critics
{

class AdaptiveHeadingCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

private:
  struct CorridorCandidate
  {
    float width;
    float angle_deg;
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  std::tuple<std::optional<float>, std::optional<float>, std::optional<float>>
  extractWidth(const std::vector<int> & counts, const std::vector<float> & bins) const;

  void updateCorridorState(
    const std::vector<CorridorCandidate> & valid_widths,
    float robot_yaw);

  void setHeadingMode(
    bool lock_heading,
    bool update_target,
    const std::optional<float> & target_yaw);

  float normalizeAngle(float angle) const;
  float shortestAngularDistance(float from, float to) const;

  bool enabled_{true};

  // common
  int power_{1};
  float threshold_to_consider_{0.5f};

  // path-angle mode
  int offset_from_furthest_{4};
  float path_angle_weight_{2.0f};
  float max_angle_to_furthest_{1.2f};
  bool forward_preference_{true};
  bool reversing_allowed_{false};

  // heading-lock mode
  float heading_lock_weight_{5.0f};
  float locked_heading_{0.0f};

  // scan / corridor params
  std::string scan_topic_{"/scan"};
  std::string global_frame_{"odom"};
  float max_scan_range_{5.0f};

  float histogram_min_{-5.0f};
  float histogram_max_{5.0f};
  float histogram_resolution_{0.05f};

  int search_angle_step_deg_{2};
  int search_angle_max_deg_{180};
  int second_axis_offset_idx_{45};
  int peak_min_count_{15};

  float corridor_width_min_{0.4f};
  float corridor_width_max_{2.0f};

  float single_corridor_lock_width_threshold_{1.15f};
  float unlock_hysteresis_low_{1.10f};
  float lock_hysteresis_high_{1.20f};

  float scan_process_period_{0.5f};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex state_mutex_;

  bool heading_locked_{false};
  bool heading_lock_initialized_{false};

  std::optional<float> last_target_yaw_;
  std::optional<float> w_curr_ema_;
  std::optional<float> w_next_ema_;
  bool has_aligned_to_single_corridor_{true};

  rclcpp::Time last_scan_process_time_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__ADAPTIVE_HEADING_CRITIC_HPP_