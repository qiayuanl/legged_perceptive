//
// Created by qiayuan on 23-1-1.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_perceptive_interface/PerceptiveLeggedPrecomputation.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

namespace legged {
PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                               const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings,
                                                               const ConvexRegionSelector& convexRegionSelector)
    : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
      info_(std::move(info)),
      mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
      convexRegionSelectorPtr_(&convexRegionSelector) {
  mappingPtr_->setPinocchioInterface(getPinocchioInterface());
  footPlacementConParameters_.resize(info_.numThreeDofContacts);
}

PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
    : LeggedRobotPreComputation(rhs),
      info_(rhs.info_),
      mappingPtr_(rhs.mappingPtr_->clone()),
      convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_) {
  mappingPtr_->setPinocchioInterface(getPinocchioInterface());
  footPlacementConParameters_.resize(info_.numThreeDofContacts);
}

void PerceptiveLeggedPrecomputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }
  LeggedRobotPreComputation::request(request, t, x, u);

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      FootPlacementConstraint::Parameter params;

      auto projection = convexRegionSelectorPtr_->getProjection(i, t);
      if (projection.regionPtr == nullptr) {  // Swing leg
        continue;
      }

      matrix_t polytopeA;
      vector_t polytopeB;
      std::tie(polytopeA, polytopeB) = getPolygonConstraint(convexRegionSelectorPtr_->getConvexPolygon(i, t));
      matrix_t p = (matrix_t(2, 3) <<  // clang-format off
                        1, 0, 0,
                        0, 1, 0).finished();  // clang-format on
      params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
      params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().head(2);

      footPlacementConParameters_[i] = params;
    }
  }

  // For SDF constraints
  const auto& model = getPinocchioInterface().getModel();
  auto& data = getPinocchioInterface().getData();
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);

  if (request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    //    pinocchio::updateGlobalPlacements(model, data);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }
}

std::pair<matrix_t, vector_t> PerceptiveLeggedPrecomputation::getPolygonConstraint(
    const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  size_t numVertices = polygon.size();
  matrix_t polytopeA = matrix_t::Zero(numVertices, 2);
  vector_t polytopeB = vector_t::Zero(numVertices);

  for (size_t i = 0; i < numVertices; i++) {
    size_t j = i + 1;
    if (j == numVertices) {
      j = 0;
    }
    size_t k = j + 1;
    if (k == numVertices) {
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
