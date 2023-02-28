//
// Created by qiayuan on 23-2-28.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <utility>

#include "legged_perceptive_controllers/FootHeightEstimation.h"

namespace legged {
FootHeightEstimation::FootHeightEstimation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics& eeKinematics,
                                           std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr)
    : KalmanFilterEstimate(std::move(pinocchioInterface), std::move(info), eeKinematics), planarTerrainPtr_(std::move(planarTerrainPtr)) {}
vector_t FootHeightEstimation::update(const ros::Time& time, const ros::Duration& period) {
  updateFootHeight();
  return KalmanFilterEstimate::update(time, period);
}

void FootHeightEstimation::updateFootHeight() {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  vector_t q(info_.generalizedCoordinatesNum);
  q.head<3>() = rbdState_.segment<3>(3);
  q.segment<3>(3) = rbdState_.head<3>();
  q.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  eeKinematics_->setPinocchioInterface(pinocchioInterface_);

  for (size_t i = 0; i < 4; i++) {
    const vector3_t pos = eeKinematics_->getPosition(vector_t())[i];
    feetHeights_[i] = planarTerrainPtr_->gridMap.atPosition("elevation_before_postprocess", grid_map::Position(pos.head(2)));
  }
}

}  // namespace legged
