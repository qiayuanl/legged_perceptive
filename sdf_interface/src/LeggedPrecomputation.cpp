//
// Created by qiayuan on 23-1-1.
//

#include <utility>

#include "sdf_interface/LeggedPrecomputation.h"

namespace legged {
LeggedPreComputation::LeggedPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                           const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings)
    : LeggedRobotPreComputation(std::move(pinocchioInterface), std::move(info), swingTrajectoryPlanner, std::move(settings)) {}

void LeggedPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  LeggedRobotPreComputation::request(request, t, x, u);

  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }
}

}  // namespace legged
