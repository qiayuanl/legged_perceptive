//
// Created by qiayuan on 23-1-1.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_legged_robot/LeggedRobotPreComputation.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/** Callback for caching and reference update */
class LeggedPreComputation : public LeggedRobotPreComputation {
 public:
  LeggedPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                       const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings);
  ~LeggedPreComputation() override = default;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
};

}  // namespace legged
