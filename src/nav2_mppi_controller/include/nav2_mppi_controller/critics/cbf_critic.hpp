#ifndef NAV2_MPPI_CONTROLLER__CRITICS__CBF_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__CBF_CRITIC_HPP_

#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mppi::critics
{

class CBFCritic : public CriticFunction
{
public:
  CBFCritic() = default;
  ~CBFCritic() override = default;

  void initialize() override;
  void score(CriticData & data) override;

protected:
  struct Point2D
  {
    float x;
    float y;
  };

  void updateFootprintGeometryCache();
  void updateLethalPointsCache();

  bool isLethalCost(unsigned char cost) const
  {
    using namespace nav2_costmap_2d;
    if (cost == LETHAL_OBSTACLE) {
      return true;
    }
    return false;
  }

  float distancePointToCircle(float px, float py, float rx, float ry) const;
  float distancePointToOrientedRect(
    float px, float py, float rx, float ry, float rtheta) const;
  float distancePointToFootprint(
    float px, float py, float rx, float ry, float rtheta) const;

protected:
  bool enabled_{true};
  bool consider_footprint_{true};
  uint8_t power_{0};
  float cost_weight_{0};
  float cbf_alpha_{0};
  float threshold_to_consider_{0};
  float threshold_dist_{0};
  float collision_cost_{0};
  float weight_{0};
  float inscribed_radius_{0};
  float rect_hx_{0};
  float rect_hy_{0};

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
    collision_checker_;

  std::vector<Point2D> lethal_points_;
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__CBF_CRITIC_HPP_

