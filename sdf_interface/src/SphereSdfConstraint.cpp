//
// Created by qiayuan on 22-12-27.
//

#include "sdf_interface/SphereSdfConstraint.h"

namespace legged {
SphereSdfConstraint::SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematicsPtr, PinocchioInterface& pinocchioInterface,
                                         std::shared_ptr<Sdf> sdfPtr)
    : StateConstraint(ConstraintOrder::Linear),
      sphereKinematicsPtr_(sphereKinematicsPtr.clone()),
      pinocchioInterface_(pinocchioInterface),
      sdfPtr_(std::move(sdfPtr)),
      numConstraints_(sphereKinematicsPtr_->getPinocchioSphereInterface().getNumSpheresInTotal()) {
  sphereKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
}

vector_t SphereSdfConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {
  vector_t value(numConstraints_);

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
    approx.dfdx.row(i) = sdfGradient * sphereApprox[i].dfdx;
  }
  return approx;
}

SphereSdfConstraint::SphereSdfConstraint(const SphereSdfConstraint& rhs)
    : StateConstraint(rhs),
      sphereKinematicsPtr_(rhs.sphereKinematicsPtr_->clone()),
      pinocchioInterface_(rhs.pinocchioInterface_),
      sdfPtr_(rhs.sdfPtr_),
      numConstraints_(rhs.numConstraints_) {
  sphereKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
}

}  // namespace legged
