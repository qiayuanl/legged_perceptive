//
// Created by qiayuan on 23-1-1.
//

#include <utility>

#include "sdf_interface/LeggedPrecomputation.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>

namespace legged {
LeggedPreComputation::LeggedPreComputation(PinocchioInterface pinocchioInterface, const CentroidalModelInfo& info,
                                           const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                           const ConvexRegionSelector& convexRegionSelector, size_t numVertices)
    : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
      convexRegionSelectorPtr_(&convexRegionSelector),
      numVertices_(numVertices) {
  footPlacementConParameters_.resize(info.numThreeDofContacts);
}

void LeggedPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  LeggedRobotPreComputation::request(request, t, x, u);

  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  // lambda to set config for foot placement constraints
  auto footPlacementConParameter = [&](size_t footIndex) {
    FootPlacementConstraint::Parameter params;

    auto projection = convexRegionSelectorPtr_->getProjection(footIndex, t);
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
    return params;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < footPlacementConParameters_.size(); i++) {
      footPlacementConParameters_[i] = footPlacementConParameter(i);
    }
  }
}

std::pair<matrix_t, vector_t> LeggedPreComputation::getPolygonConstraint(const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  matrix_t polytopeA = matrix_t::Zero(numVertices_, 2);
  vector_t polytopeB = vector_t::Zero(numVertices_);

  const auto vertices = polygon.vertices_begin();
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
    if ((polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished())(0) > polytopeB(i)) {
      polytopeA.row(i) *= -1;
      polytopeB(i) *= -1;
    }
  }

  return {polytopeA, polytopeB};
}

}  // namespace legged
