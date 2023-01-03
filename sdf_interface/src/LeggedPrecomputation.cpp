//
// Created by qiayuan on 23-1-1.
//

#include <utility>

#include "sdf_interface/LeggedPrecomputation.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>

namespace legged {
LeggedPreComputation::LeggedPreComputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                                           const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                           const ConvexRegionSelector& convexRegionSelector, const size_t numberOfVertices)
    : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
      convexRegionSelectorPtr_(&convexRegionSelector),
      numberOfVertices_(numberOfVertices) {
  footPlacementConConfigs_.resize(info.numThreeDofContacts);
}

void LeggedPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  LeggedRobotPreComputation::request(request, t, x, u);

  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for foot placement constraints
  auto footPlacementConConfig = [&](size_t footIndex) {
    FootPlacementConstraint::Config config;

    auto projection = convexRegionSelectorPtr_->getProjection(footIndex, t);
    scalar_t growthFactor = 1.05;
    const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
        projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numberOfVertices_, growthFactor);

    matrix_t polytopeA;
    vector_t polytopeB;
    std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegion);
    matrix_t p = (matrix_t(2, 4) <<  // clang-format off
                        1, 0, 0, 0,
                        0, 1, 0, 0).finished();  // clang-format on
    config.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().matrix();
    projection.regionPtr->transformPlaneToWorld.linear();
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < footPlacementConConfigs_.size(); i++) {
      footPlacementConConfigs_[i] = footPlacementConConfig(i);
    }
  }
}

std::pair<matrix_t, vector_t> LeggedPreComputation::getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  matrix_t polytopeA = matrix_t::Zero(numberOfVertices_, 2);
  vector_t polytopeB = vector_t::Zero(numberOfVertices_);

  const auto vertices = polygon.vertices_begin();
  for (size_t i = 0; i < numberOfVertices_; i++) {
    size_t j = i + 1;
    if (j == numberOfVertices_) {
      j = 0;
    }
    size_t k = j + 1;
    if (k == numberOfVertices_) {
      k = 0;
    }
    const auto point_a = polygon.vertex(i);
    const auto point_b = polygon.vertex(j);
    const auto point_c = polygon.vertex(k);

    polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
    polytopeB(i) = point_a.x() * point_b.y() - point_a.y() * point_b.x();
    if ((polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished())(0) > polytopeB(i)) {
      polytopeA.row(i) *= -1;
      polytopeB(i) *= -1;
    }
  }

  return {polytopeA, polytopeB};
}

}  // namespace legged
