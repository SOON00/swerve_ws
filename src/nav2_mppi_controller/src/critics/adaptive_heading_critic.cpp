#include "nav2_mppi_controller/critics/adaptive_heading_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"

namespace mppi::critics
{

void AdaptiveHeadingCritic::initialize()
{
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  float vx_min;
  getParentParam(vx_min, "vx_min", -0.35f);

  if (std::fabs(vx_min) < 1e-6f) {
    reversing_allowed_ = false;
  } else if (vx_min < 0.0f) {
    reversing_allowed_ = true;
  } else {
    reversing_allowed_ = false;
  }

  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(enabled_, "enabled", true);

  // common
  getParam(power_, "cost_power", 1);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);

  // path-angle mode
  getParam(offset_from_furthest_, "offset_from_furthest", 4);
  getParam(path_angle_weight_, "path_angle_weight", 2.0f);
  getParam(max_angle_to_furthest_, "max_angle_to_furthest", 1.2f);
  getParam(forward_preference_, "forward_preference", true);

  // heading-lock mode
  getParam(heading_lock_weight_, "heading_lock_weight", 5.0f);
  getParam(locked_heading_, "locked_heading", 0.0f);

  // scan / corridor
  getParam(scan_topic_, "scan_topic", std::string("/scan"));
  getParam(global_frame_, "global_frame", std::string("odom"));
  getParam(max_scan_range_, "max_scan_range", 5.0f);

  getParam(histogram_min_, "histogram_min", -5.0f);
  getParam(histogram_max_, "histogram_max", 5.0f);
  getParam(histogram_resolution_, "histogram_resolution", 0.05f);

  getParam(search_angle_step_deg_, "search_angle_step_deg", 2);
  getParam(search_angle_max_deg_, "search_angle_max_deg", 180);
  getParam(second_axis_offset_idx_, "second_axis_offset_idx", 45);
  getParam(peak_min_count_, "peak_min_count", 15);

  getParam(corridor_width_min_, "corridor_width_min", 0.4f);
  getParam(corridor_width_max_, "corridor_width_max", 2.0f);

  getParam(
    single_corridor_lock_width_threshold_,
    "single_corridor_lock_width_threshold",
    1.15f);
  getParam(unlock_hysteresis_low_, "unlock_hysteresis_low", 1.10f);
  getParam(lock_hysteresis_high_, "lock_hysteresis_high", 1.20f);

  getParam(scan_process_period_, "scan_process_period", 0.5f);

  if (!reversing_allowed_) {
    forward_preference_ = true;
  }

  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error(
            "Failed to lock parent node in AdaptiveHeadingCritic::initialize()");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&AdaptiveHeadingCritic::scanCallback, this, std::placeholders::_1));

  last_scan_process_time_ = node->now();

  RCLCPP_INFO(
    logger_,
    "AdaptiveHeadingCritic instantiated. power=%u, path_angle_weight=%.3f, "
    "heading_lock_weight=%.3f, reversing=%s, scan_topic=%s, global_frame=%s",
    power_,
    path_angle_weight_,
    heading_lock_weight_,
    reversing_allowed_ ? "allowed" : "not allowed",
    scan_topic_.c_str(),
    global_frame_.c_str());
}

float AdaptiveHeadingCritic::normalizeAngle(float angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

float AdaptiveHeadingCritic::shortestAngularDistance(float from, float to) const
{
  return normalizeAngle(to - from);
}

std::tuple<std::optional<float>, std::optional<float>, std::optional<float>>
AdaptiveHeadingCritic::extractWidth(
  const std::vector<int> & counts,
  const std::vector<float> & bins) const
{
  std::vector<std::pair<int, float>> peaks;

  for (size_t j = 1; j + 1 < counts.size(); ++j) {
    if (
      counts[j] > counts[j - 1] &&
      counts[j] > counts[j + 1] &&
      counts[j] > peak_min_count_)
    {
      peaks.emplace_back(counts[j], bins[j]);
    }
  }

  if (peaks.size() >= 2) {
    std::sort(
      peaks.begin(), peaks.end(),
      [](const auto & a, const auto & b) {
        return a.first > b.first;
      });

    const float r1 = peaks[0].second;
    const float r2 = peaks[1].second;
    return {std::fabs(r1 - r2), std::min(r1, r2), std::max(r1, r2)};
  }

  return {std::nullopt, std::nullopt, std::nullopt};
}

void AdaptiveHeadingCritic::setHeadingMode(
  bool lock_heading,
  bool update_target,
  const std::optional<float> & target_yaw)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  heading_locked_ = lock_heading;
  heading_lock_initialized_ = true;

  if (update_target && target_yaw.has_value()) {
    locked_heading_ = normalizeAngle(*target_yaw);
    last_target_yaw_ = locked_heading_;
  }
}

void AdaptiveHeadingCritic::updateCorridorState(
  const std::vector<CorridorCandidate> & valid_widths,
  float robot_yaw)
{
  if (valid_widths.size() == 1) {
    const float w_single = valid_widths[0].width;
    const float angle_single = valid_widths[0].angle_deg;

    if (!w_curr_ema_.has_value()) {
      w_curr_ema_ = w_single;
    } else {
      if (
        w_next_ema_.has_value() &&
        std::fabs(w_single - *w_next_ema_) < std::fabs(w_single - *w_curr_ema_))
      {
        RCLCPP_INFO(
          logger_,
          "Single corridor detected: next corridor became current corridor.");
        w_curr_ema_ = w_single;
        w_next_ema_.reset();
        has_aligned_to_single_corridor_ = false;
      } else {
        w_curr_ema_ = 0.6f * (*w_curr_ema_) + 0.4f * w_single;
      }
    }

    if (!has_aligned_to_single_corridor_) {
      // Python 로직 동일
      const float local_yaw_1 =
        static_cast<float>(M_PI / 180.0) * (angle_single + 90.0f);
      const float local_yaw_2 =
        static_cast<float>(M_PI / 180.0) * (angle_single - 90.0f);

      const float norm_yaw_1 = normalizeAngle(local_yaw_1);
      const float norm_yaw_2 = normalizeAngle(local_yaw_2);

      const float best_local_yaw =
        (std::fabs(norm_yaw_1) < std::fabs(norm_yaw_2)) ? norm_yaw_1 : norm_yaw_2;

      float target_yaw_calc = normalizeAngle(robot_yaw + best_local_yaw);

      // Python의 pi ambiguity handling 동일
      if (std::fabs(std::fabs(target_yaw_calc) - static_cast<float>(M_PI)) < 0.1f) {
        if (robot_yaw >= 0.0f) {
          target_yaw_calc = std::fabs(target_yaw_calc);
        } else {
          target_yaw_calc = -std::fabs(target_yaw_calc);
        }
      } else if (std::fabs(target_yaw_calc) < 0.001f) {
        target_yaw_calc = 0.0f;
      }

      if (*w_curr_ema_ <= single_corridor_lock_width_threshold_) {
        setHeadingMode(true, true, target_yaw_calc);
        RCLCPP_INFO(
          logger_,
          "[AdaptiveHeadingCritic] Heading lock ON | target updated = %.3f rad",
          target_yaw_calc);
      } else {
        setHeadingMode(true, false, std::nullopt);
        RCLCPP_INFO(
          logger_,
          "[AdaptiveHeadingCritic] Heading lock ON | target unchanged");
      }

      has_aligned_to_single_corridor_ = true;
    }

    return;
  }

  if (valid_widths.size() == 2) {
    has_aligned_to_single_corridor_ = false;

    const auto & a = valid_widths[0];
    const auto & b = valid_widths[1];

    if (!w_curr_ema_.has_value()) {
      if (a.width < b.width) {
        w_curr_ema_ = a.width;
        w_next_ema_ = b.width;
      } else {
        w_curr_ema_ = b.width;
        w_next_ema_ = a.width;
      }
    } else {
      float c_raw, n_raw;
      if (std::fabs(a.width - *w_curr_ema_) < std::fabs(b.width - *w_curr_ema_)) {
        c_raw = a.width;
        n_raw = b.width;
      } else {
        c_raw = b.width;
        n_raw = a.width;
      }

      w_curr_ema_ = 0.6f * (*w_curr_ema_) + 0.4f * c_raw;

      if (!w_next_ema_.has_value()) {
        w_next_ema_ = n_raw;
      } else {
        w_next_ema_ = 0.6f * (*w_next_ema_) + 0.4f * n_raw;
      }
    }

    if (w_next_ema_.has_value()) {
      bool desired_lock = (*w_next_ema_ > single_corridor_lock_width_threshold_);

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (heading_lock_initialized_) {
          if (heading_locked_) {
            desired_lock = !(*w_next_ema_ < unlock_hysteresis_low_);
          } else {
            desired_lock = (*w_next_ema_ > lock_hysteresis_high_);
          }
        }
      }

      setHeadingMode(desired_lock, false, std::nullopt);

      RCLCPP_INFO(
        logger_,
        "[AdaptiveHeadingCritic] Two corridors | curr=%.3f next=%.3f lock=%s",
        w_curr_ema_.value_or(-1.0f),
        w_next_ema_.value_or(-1.0f),
        desired_lock ? "ON" : "OFF");
    }
  }
}

void AdaptiveHeadingCritic::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  auto node = parent_.lock();
  if (!node) {
    return;
  }

  const auto now = node->now();

  if ((now - last_scan_process_time_).seconds() < scan_process_period_) {
    return;
  }

  float robot_yaw = 0.0f;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      global_frame_,
      msg->header.frame_id,
      tf2::TimePointZero);

    robot_yaw = static_cast<float>(tf2::getYaw(tf.transform.rotation));
  } catch (const std::exception & e) {
    RCLCPP_WARN(
    logger_,
    "AdaptiveHeadingCritic TF lookup failed: %s",
    e.what());
    return;
  }

  const size_t n = msg->ranges.size();
  if (n == 0) {
    return;
  }

  std::vector<float> xs;
  std::vector<float> ys;
  xs.reserve(n);
  ys.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const float r = msg->ranges[i];
    const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;

    if (r > msg->range_min && r < max_scan_range_ && std::isfinite(r)) {
      xs.push_back(r * std::cos(angle));
      ys.push_back(r * std::sin(angle));
    }
  }

  if (xs.empty()) {
    return;
  }

  std::vector<float> search_angles_rad;
  for (int deg = 0; deg < search_angle_max_deg_; deg += search_angle_step_deg_) {
    search_angles_rad.push_back(
      static_cast<float>(deg) * static_cast<float>(M_PI / 180.0));
  }

  std::vector<float> bins;
  for (float b = histogram_min_; b < histogram_max_; b += histogram_resolution_) {
    bins.push_back(b);
  }

  if (bins.size() < 2) {
    return;
  }

  std::vector<std::vector<int>> all_counts(search_angles_rad.size());
  std::vector<int> max_peaks(search_angles_rad.size(), 0);

  for (size_t ai = 0; ai < search_angles_rad.size(); ++ai) {
    const float ca = std::cos(search_angles_rad[ai]);
    const float sa = std::sin(search_angles_rad[ai]);

    std::vector<int> counts(bins.size() - 1, 0);

    for (size_t i = 0; i < xs.size(); ++i) {
      const float proj = xs[i] * ca + ys[i] * sa;
      const int bin_idx = static_cast<int>((proj - histogram_min_) / histogram_resolution_);
      if (bin_idx >= 0 && bin_idx < static_cast<int>(counts.size())) {
        counts[bin_idx]++;
      }
    }

    max_peaks[ai] = *std::max_element(counts.begin(), counts.end());
    all_counts[ai] = std::move(counts);
  }

  const auto best_it = std::max_element(max_peaks.begin(), max_peaks.end());
  if (best_it == max_peaks.end()) {
    return;
  }

  const int best_idx1 = static_cast<int>(std::distance(max_peaks.begin(), best_it));
  const int axis_count = static_cast<int>(search_angles_rad.size());
  if (axis_count <= 0) {
    return;
  }

  const int best_idx2 = (best_idx1 + second_axis_offset_idx_) % axis_count;

  std::vector<CorridorCandidate> valid_widths;

  const auto [w1, r1_min, r1_max] = extractWidth(all_counts[best_idx1], bins);
  (void)r1_min;
  (void)r1_max;

  const auto [w2, r2_min, r2_max] = extractWidth(all_counts[best_idx2], bins);
  (void)r2_min;
  (void)r2_max;

  if (w1.has_value() && *w1 >= corridor_width_min_ && *w1 <= corridor_width_max_) {
    valid_widths.push_back(
      CorridorCandidate{
        *w1,
        static_cast<float>(best_idx1 * search_angle_step_deg_)
      });
  }

  if (w2.has_value() && *w2 >= corridor_width_min_ && *w2 <= corridor_width_max_) {
    valid_widths.push_back(
      CorridorCandidate{
        *w2,
        static_cast<float>(best_idx2 * search_angle_step_deg_)
      });
  }

  if (!valid_widths.empty()) {
    updateCorridorState(valid_widths, robot_yaw);
    last_scan_process_time_ = now;
  }
}

void AdaptiveHeadingCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(
        threshold_to_consider_,
        data.state.pose.pose,
        data.path))
  {
    return;
  }

  bool heading_locked_local;
  float locked_heading_local;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    heading_locked_local = heading_locked_;
    locked_heading_local = locked_heading_;
  }

  // heading lock mode
  if (heading_locked_local) {
    auto angle_errors = xt::abs(
      utils::shortest_angular_distance(
        data.trajectories.yaws,
        locked_heading_local));

    data.costs += xt::pow(
      xt::mean(angle_errors, {1}, immediate) * heading_lock_weight_,
      power_);

    return;
  }

  // path-angle mode
  utils::setPathFurthestPointIfNotSet(data);

  auto offseted_idx = std::min(
    *data.furthest_reached_path_point + offset_from_furthest_,
    data.path.x.shape(0) - 1);

  const float goal_x = xt::view(data.path.x, offseted_idx);
  const float goal_y = xt::view(data.path.y, offseted_idx);

  if (
    utils::posePointAngle(
      data.state.pose.pose,
      goal_x,
      goal_y,
      forward_preference_) < max_angle_to_furthest_)
  {
    return;
  }

  auto yaws_between_points = xt::atan2(
    goal_y - data.trajectories.y,
    goal_x - data.trajectories.x);

  auto yaws = xt::abs(
    utils::shortest_angular_distance(data.trajectories.yaws, yaws_between_points));

  if (reversing_allowed_ && !forward_preference_) {
    const auto yaws_between_points_corrected = xt::where(
      yaws < M_PI_2,
      yaws_between_points,
      utils::normalize_angles(yaws_between_points + M_PI));

    const auto corrected_yaws = xt::abs(
      utils::shortest_angular_distance(
        data.trajectories.yaws,
        yaws_between_points_corrected));

    data.costs += xt::pow(
      xt::mean(corrected_yaws, {1}, immediate) * path_angle_weight_,
      power_);
  } else {
    data.costs += xt::pow(
      xt::mean(yaws, {1}, immediate) * path_angle_weight_,
      power_);
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::AdaptiveHeadingCritic,
  mppi::critics::CriticFunction)