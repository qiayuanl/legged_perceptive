//
// Created by qiayuan on 23-1-1.
//

#include "sdf_interface/constraint/FootPlacementConstraint.h"
#include "sdf_interface/LeggedPrecomputation.h"

namespace legged {
FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex,
                                                 size_t numVertices)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex),
      numVertices_(numVertices) {}

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_),
      numVertices_(rhs.numVertices_) {}

bool FootPlacementConstraint::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

vector_t FootPlacementConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& preComp) const {
  const auto param = cast<LeggedPreComputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
  return param.a * endEffectorKinematicsPtr_->getPosition(state)[contactPointIndex_] + param.b;
}

VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t /*time*/, const vector_t& state,
                                                                                  const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numVertices_, state.size(), 0);
  const auto param = cast<LeggedPreComputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];

  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  approx.f.noalias() += param.a * positionApprox.f + param.b;
  approx.dfdx.noalias() += param.a * positionApprox.dfdx;
  return approx;
}

}  // namespace legged
