#include "nav2_mppi_controller/critics/heading_lock_critic.hpp"
#include <math.h>
#include <pluginlib/class_list_macros.hpp>

namespace mppi::critics
{

void HeadingLockCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0); // 헤딩 고정을 위해 가중치를 높게 설정 권장
  getParam(locked_heading_, "locked_heading", 0.0); // YAML에서 설정 가능

  RCLCPP_INFO(
    logger_,
    "HeadingLockCritic instantiated with %d power, %f weight, and %f target heading.",
    power_, weight_, locked_heading_);
}

void HeadingLockCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;
  if (!enabled_) return;

  // getParam 호출 없이 바로 멤버 변수 사용 (Nav2가 백그라운드에서 업데이트해줌)
  auto angle_errors = xt::abs(utils::shortest_angular_distance(
    data.trajectories.yaws, 
    locked_heading_ 
  ));

  data.costs += xt::pow(
    xt::mean(angle_errors, {1}, immediate) * weight_, 
    power_
  );
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::HeadingLockCritic,
  mppi::critics::CriticFunction)
