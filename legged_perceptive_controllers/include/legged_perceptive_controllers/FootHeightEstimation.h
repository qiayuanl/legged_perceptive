//
// Created by qiayuan on 23-2-28.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <legged_estimation/LinearKalmanFilter.h>

namespace legged {

class FootHeightEstimation : public KalmanFilterEstimate {
 public:
  FootHeightEstimation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics,
                       std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr);

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

 protected:
  void updateFootHeight();

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
};

}  // namespace legged
