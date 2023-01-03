//
// Created by qiayuan on 23-1-1.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <ocs2_legged_robot/LeggedRobotPreComputation.h>
#include <sdf_interface/ConvexRegionSelector.h>
#include <sdf_interface/constraint/FootPlacementConstraint.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/** Callback for caching and reference update */
class LeggedPreComputation : public LeggedRobotPreComputation {
 public:
  LeggedPreComputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                       const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                       const ConvexRegionSelector& convexRegionSelector, const size_t numberOfVertices);
  ~LeggedPreComputation() override = default;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

 private:
  std::pair<matrix_t, vector_t> getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  size_t numberOfVertices_;
  const ConvexRegionSelector* convexRegionSelectorPtr_;
  std::vector<FootPlacementConstraint::Config> footPlacementConConfigs_;
};

}  // namespace legged
