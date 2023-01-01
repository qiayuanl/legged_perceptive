//
// Created by qiayuan on 22-12-27.
//

#include <pinocchio/fwd.hpp>

#include "sdf_interface/SphereSdfConstraint.h"

#include <pinocchio/algorithm/frames.hpp>

namespace legged {
SphereSdfConstraint::SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematics, PinocchioInterface& pinocchioInterface,
                                         const PinocchioStateInputMapping<scalar_t>& mapping,
                                         std::shared_ptr<grid_map::SignedDistanceField> sdfPtr)
    : StateConstraint(ConstraintOrder::Linear),
      sphereKinematicsPtr_(sphereKinematics.clone()),
      pinocchioInterface_(pinocchioInterface),
      mappingPtr_(mapping.clone()),
      sdfPtr_(std::move(sdfPtr)),
      numConstraints_(sphereKinematicsPtr_->getPinocchioSphereInterface().getNumSpheresInTotal()) {
  sphereKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
}

vector_t SphereSdfConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  vector_t value(numConstraints_);

  vector_t q = mappingPtr_->getPinocchioJointPosition(state);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  auto position = sphereKinematicsPtr_->getPosition(state);
  auto radius = sphereKinematicsPtr_->getPinocchioSphereInterface().getSphereRadii();
  for (int i = 0; i < numConstraints_; ++i) {
    value(i) = sdfPtr_->value(grid_map::Position3(position[i])) - radius[i];
  }
  return value;
}

VectorFunctionLinearApproximation SphereSdfConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numConstraints_, state.size(), 0);
  approx.f = getValue(time, state, preComp);

  auto position = sphereKinematicsPtr_->getPosition(state);
  auto sphereApprox = sphereKinematicsPtr_->getPositionLinearApproximation(state);

  for (int i = 0; i < numConstraints_; ++i) {
    vector_t sdfGradient = sdfPtr_->derivative(grid_map::Position3(position[i]));
    approx.dfdx.row(i) = sdfGradient.transpose() * sphereApprox[i].dfdx;
  }
  return approx;
}

SphereSdfConstraint::SphereSdfConstraint(const SphereSdfConstraint& rhs)
    : StateConstraint(rhs),
      sphereKinematicsPtr_(rhs.sphereKinematicsPtr_->clone()),
      pinocchioInterface_(rhs.pinocchioInterface_),
      mappingPtr_(rhs.mappingPtr_->clone()),
      sdfPtr_(rhs.sdfPtr_),
      numConstraints_(rhs.numConstraints_) {
  sphereKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
}

}  // namespace legged
