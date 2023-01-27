//
// Created by qiayuan on 23-1-1.
//

#pragma once

#include <legged_interface/LeggedRobotPreComputation.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <legged_perceptive_interface/ConvexRegionSelector.h>
#include <legged_perceptive_interface/constraint/FootPlacementConstraint.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/** Callback for caching and reference update */
class PerceptiveLeggedPrecomputation : public LeggedRobotPreComputation {
 public:
  PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                 const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                 const ConvexRegionSelector& convexRegionSelector, size_t numVertices);
  ~PerceptiveLeggedPrecomputation() override = default;

  PerceptiveLeggedPrecomputation* clone() const override { return new PerceptiveLeggedPrecomputation(*this); }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  const std::vector<FootPlacementConstraint::Parameter>& getFootPlacementConParameters() const { return footPlacementConParameters_; }

  const std::vector<convex_plane_decomposition::CgalPolygon2d>& getConvexRegions() const { return convexRegions_; }

  PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs);

 private:
  std::pair<matrix_t, vector_t> getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const;

  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;

  // ConvexRegionSelector
  size_t numVertices_;
  const ConvexRegionSelector* convexRegionSelectorPtr_;

  std::vector<FootPlacementConstraint::Parameter> footPlacementConParameters_;
  std::vector<convex_plane_decomposition::CgalPolygon2d> convexRegions_;
};

}  // namespace legged
