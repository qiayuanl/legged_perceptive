//
// Created by qiayuan on 23-1-1.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <legged_perceptive_interface/ConvexRegionSelector.h>
#include <legged_perceptive_interface/constraint/FootPlacementConstraint.h>
#include <ocs2_legged_robot/LeggedRobotPreComputation.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/** Callback for caching and reference update */
class LeggedPreComputation : public PreComputation {
 public:
  LeggedPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                       const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                       const ConvexRegionSelector& convexRegionSelector, size_t numVertices);
  ~LeggedPreComputation() override = default;

  LeggedPreComputation* clone() const override;

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<EndEffectorLinearConstraint::Config>& getEeNormalVelocityConstraintConfigs() const { return eeNormalVelConConfigs_; }

  const std::vector<FootPlacementConstraint::Parameter>& getFootPlacementConParameters() const { return footPlacementConParameters_; }

  const std::vector<convex_plane_decomposition::CgalPolygon2d>& getConvexRegions() const { return convexRegions_; }

  LeggedPreComputation(const LeggedPreComputation& other) = default;

 private:
  std::pair<matrix_t, vector_t> getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  // SwingTrajectoryPlanner
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  const ModelSettings settings_;

  std::vector<EndEffectorLinearConstraint::Config> eeNormalVelConConfigs_;

  // ConvexRegionSelector
  size_t numVertices_;
  const ConvexRegionSelector* convexRegionSelectorPtr_;

  std::vector<FootPlacementConstraint::Parameter> footPlacementConParameters_;
  std::vector<convex_plane_decomposition::CgalPolygon2d> convexRegions_;
};

}  // namespace legged
