//
// Created by qiayuan on 23-1-1.
//

#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class FootPlacementConstraint final : public StateConstraint {
 public:
  struct Config {
    matrix_t a;
    vector_t b;
    vector_t s;
  };

  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                          const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex);

  ~FootPlacementConstraint() override = default;
  FootPlacementConstraint* clone() const override { return new FootPlacementConstraint(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  FootPlacementConstraint(const FootPlacementConstraint& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  const size_t contactPointIndex_;
};

}  // namespace legged