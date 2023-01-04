//
// Created by qiayuan on 23-1-1.
//

#include <utility>

#include "sdf_interface/LeggedPrecomputation.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>

namespace legged {
LeggedPreComputation::LeggedPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                           const ConvexRegionSelector& convexRegionSelector, size_t numVertices)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      settings_(std::move(settings)),
      convexRegionSelectorPtr_(&convexRegionSelector),
      numVertices_(numVertices) {
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  footPlacementConParameters_.resize(info_.numThreeDofContacts);
  convexRegions_.resize(info_.numThreeDofContacts);
}

LeggedPreComputation* LeggedPreComputation::clone() const {
  return new LeggedPreComputation(*this);
}

void LeggedPreComputation::request(RequestSet request, scalar_t t, const vector_t& /*x*/, const vector_t& /*u*/) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
      config.b(0) -= settings_.positionErrorGain * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, settings_.positionErrorGain).finished();
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
    }
  }

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      FootPlacementConstraint::Parameter params;

      auto projection = convexRegionSelectorPtr_->getProjection(i, t);
      if (projection.regionPtr == nullptr) {  // Swing leg
        continue;
      }

      scalar_t growthFactor = 1.05;
      const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
          projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor);

      matrix_t polytopeA;
      vector_t polytopeB;
      std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegion);
      matrix_t p = (matrix_t(2, 3) <<  // clang-format off
                        1, 0, 0,
                        0, 1, 0).finished();  // clang-format on
      params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
      params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().head(2);

      footPlacementConParameters_[i] = params;
      convexRegions_[i] = convexRegion;
    }
  }
}

std::pair<matrix_t, vector_t> LeggedPreComputation::getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  matrix_t polytopeA = matrix_t::Zero(numVertices_, 2);
  vector_t polytopeB = vector_t::Zero(numVertices_);

  for (size_t i = 0; i < numVertices_; i++) {
    size_t j = i + 1;
    if (j == numVertices_) {
      j = 0;
    }
    size_t k = j + 1;
    if (k == numVertices_) {
      k = 0;
    }
    const auto point_a = polygon.vertex(i);
    const auto point_b = polygon.vertex(j);
    const auto point_c = polygon.vertex(k);

    polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
    polytopeB(i) = point_a.y() * point_b.x() - point_a.x() * point_b.y();
    if (polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished() + polytopeB(i) < 0) {
      polytopeA.row(i) *= -1;
      polytopeB(i) *= -1;
    }
  }

  return {polytopeA, polytopeB};
}

}  // namespace legged
