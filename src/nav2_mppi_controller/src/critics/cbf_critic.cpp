#include "nav2_mppi_controller/critics/cbf_critic.hpp"
#include "nav2_core/exceptions.hpp"


namespace mppi::critics
{

void CBFCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(enabled_, "enabled", false);
  getParam(consider_footprint_, "consider_footprint", true);
  getParam(power_, "cost_power", 1);
  getParam(cost_weight_, "cost_weight", 5.0);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5);
  getParam(threshold_dist_, "threshold_dist", 0.05);
  getParam(cbf_alpha_, "cbf_alpha", 0.7);
  getParam(collision_cost_, "collision_cost", 10000.0);

  weight_ = 1000.0f;

  collision_checker_.setCostmap(costmap_);

  updateFootprintGeometryCache();

  RCLCPP_INFO(
    logger_,
    "%s instantiated. consider_footprint=%s power=%u cost_weight=%.3f"
    "threshold_to_consider=%.3f threshold_dist=%.3f cbf_alpha=%.3f"
    "collision_cost=%.1f",
    name_.c_str(),
    consider_footprint_ ? "true" : "false",
    power_, cost_weight_,
    threshold_to_consider_, threshold_dist_, cbf_alpha_, collision_cost_);
}

void CBFCritic::updateFootprintGeometryCache()
{
  inscribed_radius_ = static_cast<float>(
    costmap_ros_->getLayeredCostmap()->getInscribedRadius());

  rect_hx_ = 0.0f;
  rect_hy_ = 0.0f;

  const auto fp = costmap_ros_->getRobotFootprint();
  for (const auto & p : fp) {
    rect_hx_ = std::max(rect_hx_, std::fabs(static_cast<float>(p.x)));
    rect_hy_ = std::max(rect_hy_, std::fabs(static_cast<float>(p.y)));
  }

  if (rect_hx_ <= 1e-6f && rect_hy_ <= 1e-6f) {
    rect_hx_ = inscribed_radius_;
    rect_hy_ = inscribed_radius_;
    RCLCPP_WARN(
      logger_,
      "%s: footprint seems empty/degenerate; using inscribed_radius for rect extents.",
      name_.c_str());
  }
}

void CBFCritic::updateLethalPointsCache()
{
  lethal_points_.clear();
  lethal_points_.reserve(256);

  const auto * cm = costmap_;
  if (!cm) {
    return;
  }

  const unsigned int size_x = cm->getSizeInCellsX();
  const unsigned int size_y = cm->getSizeInCellsY();

  for (unsigned int mx = 0; mx < size_x; ++mx) {
    for (unsigned int my = 0; my < size_y; ++my) {
      const unsigned char c = cm->getCost(mx, my);
      if (!isLethalCost(c)) {
        continue;
      }

      double wx, wy;
      cm->mapToWorld(mx, my, wx, wy);
      lethal_points_.push_back(Point2D{
        static_cast<float>(wx),
        static_cast<float>(wy)});
    }
  }
}

float CBFCritic::distancePointToCircle(float px, float py, float rx, float ry) const
{
  const float dx = px - rx;
  const float dy = py - ry;
  const float d = std::hypot(dx, dy);
  return std::max(0.0f, d - inscribed_radius_);
}

float CBFCritic::distancePointToOrientedRect(
  float px, float py, float rx, float ry, float rtheta) const
{
  const float dx = px - rx;
  const float dy = py - ry;

  const float c = std::cos(-rtheta);
  const float s = std::sin(-rtheta);

  const float x_r = c * dx - s * dy;
  const float y_r = s * dx + c * dy;

  const float qx = std::fabs(x_r) - rect_hx_;
  const float qy = std::fabs(y_r) - rect_hy_;

  const float ox = std::max(qx, 0.0f);
  const float oy = std::max(qy, 0.0f);

  return std::hypot(ox, oy);
}

float CBFCritic::distancePointToFootprint(
  float px, float py, float rx, float ry, float rtheta) const
{
  if (!consider_footprint_) {
    return distancePointToCircle(px, py, rx, ry);
  }

  return distancePointToOrientedRect(px, py, rx, ry, rtheta);
}

void CBFCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  if (utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.path))
  {
    return;
  }

  updateLethalPointsCache();

  if (lethal_points_.empty()) {
    return;
  }

  const size_t batch = data.trajectories.x.shape(0);
  const size_t traj_len = data.trajectories.x.shape(1);

  auto && cbf_cost = xt::xtensor<float, 1>::from_shape({batch});
  cbf_cost.fill(0.0f);

  bool all_invalid = true;

  for (size_t i = 0; i < batch; ++i) {
    bool invalid = false;
    float sum_cost = 0.0f;

    // h_0 from first pose
    const float rx0 = data.trajectories.x(i, 0);
    const float ry0 = data.trajectories.y(i, 0);
    const float rt0 = data.trajectories.yaws(i, 0);

    float min_dist_prev = std::numeric_limits<float>::infinity();
    for (const auto & p : lethal_points_) {
      const float d = distancePointToFootprint(p.x, p.y, rx0, ry0, rt0);
      if (d < min_dist_prev) {
        min_dist_prev = d;
      }
      if (min_dist_prev <= threshold_dist_) {
        break;
      }
    }

    if (!(min_dist_prev > threshold_dist_)) {
      cbf_cost(i) = collision_cost_;

      // std::cout << std::fixed << std::setprecision(4)
      //           << "[CBFCritic] traj[" << i << "] INVALID at step 0"
      //           << " | min_dist=" << min_dist_prev
      //           << " <= threshold_dist=" << threshold_dist_
      //           << " | collision_cost=" << collision_cost_
      //           << std::endl;
      continue;
    }

    // JAX-style barrier:
    // h(x) = distance_to_obstacle - threshold_dist
    float h_prev = weight_ * (min_dist_prev - threshold_dist_);

    float worst_violation = 0.0f;
    float min_dist_all = min_dist_prev;

    for (size_t j = 1; j < traj_len; ++j) {
      const float rx = data.trajectories.x(i, j);
      const float ry = data.trajectories.y(i, j);
      const float rt = data.trajectories.yaws(i, j);

      float min_dist = std::numeric_limits<float>::infinity();
      for (const auto & p : lethal_points_) {
        const float d = distancePointToFootprint(p.x, p.y, rx, ry, rt);
        if (d < min_dist) {
          min_dist = d;
        }
        if (min_dist <= threshold_dist_) {
          break;
        }
      }

      min_dist_all = std::min(min_dist_all, min_dist);

      if (!(min_dist > threshold_dist_)) {
        invalid = true;

        // std::cout << std::fixed << std::setprecision(4)
        //           << "[CBFCritic] traj[" << i << "] INVALID at step " << j
        //           << " | min_dist=" << min_dist
        //           << " <= threshold_dist=" << threshold_dist_
        //           << " | collision_cost=" << collision_cost_
        //           << std::endl;
        break;
      }

      float h_curr = weight_ * (min_dist - threshold_dist_);

      // JAX-style:
      // cbf_cost = weight * clip(-h_{k+1} + alpha * h_k, 0, inf)
      const float violation = std::max(0.0f, -h_curr + cbf_alpha_ * h_prev);

      worst_violation = std::max(worst_violation, violation);
      sum_cost += violation;

      h_prev = h_curr;
    }

    if (invalid) {
      cbf_cost(i) = collision_cost_;
    } else {
      cbf_cost(i) = sum_cost;
      all_invalid = false;

      // const float avg_cost = sum_cost / (traj_len - 1);

      // std::cout << std::fixed << std::setprecision(4)
      //           << "[CBFCritic] traj[" << i << "]"
      //           << " | min_dist=" << min_dist_all
      //           << " | worst_violation=" << worst_violation
      //           << " | sum_cost=" << sum_cost
      //           << " | avg_cost=" << avg_cost
      //           << std::endl;
    }
  }

  data.costs += xt::pow((cost_weight_ * (cbf_cost / traj_len)), power_);
  data.fail_flag = all_invalid;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CBFCritic,
  mppi::critics::CriticFunction)