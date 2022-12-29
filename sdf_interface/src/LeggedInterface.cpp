//
// Created by qiayuan on 22-12-27.
//

#include "sdf_interface/LeggedInterface.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>

namespace legged {
void SphereSdfLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                          const std::string& referenceFile, bool verbose) {
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  sdfPrt_ = std::make_shared<Sdf>("elevation");

  PinocchioSphereInterface sphereInterface(getPinocchioInterface(), {"base"}, {0.1}, 0.5);

  CentroidalModelPinocchioMapping pinocchioMapping(getCentroidalModelInfo());
  auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(sphereInterface, pinocchioMapping);

  std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(new SphereSdfConstraint(*sphereKinematicsPtr, getPinocchioInterface(), sdfPrt_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));
  getOptimalControlProblem().stateSoftConstraintPtr->add(
      "base_sdf_constraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
}

}  // namespace legged
