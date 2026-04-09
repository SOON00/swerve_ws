#ifndef NAV2_MPPI_CONTROLLER__CRITICS__HEADING_LOCK_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__HEADING_LOCK_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::HeadingLockCritic
 * @brief 로봇이 이동 방향과 관계없이 특정 절대 각도를 유지하도록 강제하는 크리틱
 */
class HeadingLockCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{0};
  float weight_{0};
  float locked_heading_{0.0};      // 유지하고 싶은 목표 각도 (Radian)
};

}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__HEADING_LOCK_CRITIC_HPP_
