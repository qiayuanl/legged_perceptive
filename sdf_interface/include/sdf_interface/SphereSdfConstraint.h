//
// Created by qiayuan on 22-12-27.
//

#pragma once

#include <memory>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>
#include <sdf_interface/Sdf.h>

namespace legged {

using namespace ocs2;

class SphereSdfConstraint final : public ocs2::StateConstraint {
 public:
  SphereSdfConstraint(const PinocchioSphereKinematics& sphereKinematics, PinocchioInterface& pinocchioInterface,
                      std::shared_ptr<Sdf> sdfPtr);

  /** Default destructor */
  ~SphereSdfConstraint() override = default;

  SphereSdfConstraint* clone() const override { return new SphereSdfConstraint(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return numConstraints_; }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;

 private:
  SphereSdfConstraint(const SphereSdfConstraint& rhs);

  std::unique_ptr<PinocchioSphereKinematics> sphereKinematicsPtr_;
  PinocchioInterface& pinocchioInterface_;
  std::shared_ptr<Sdf> sdfPtr_;
  size_t numConstraints_{};
};

}  // namespace legged