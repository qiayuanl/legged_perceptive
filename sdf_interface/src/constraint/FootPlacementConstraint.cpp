//
// Created by qiayuan on 23-1-1.
//

#include "sdf_interface/constraint/FootPlacementConstraint.h"

namespace legged {
FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      contactPointIndex_(contactPointIndex) {}

FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

bool FootPlacementConstraint::isActive(scalar_t time) const {
  bool contact = referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  return !contact;
}

vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const {
  return ocs2::vector_t();
}

VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComp) const {
  return StateConstraint::getLinearApproximation(time, state, preComp);
}

}  // namespace legged
