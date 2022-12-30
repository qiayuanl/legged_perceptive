//
// Created by qiayuan on 22-12-27.
//

#include "sdf_interface/LeggedInterface.h"

#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>

namespace legged {
void SphereSdfLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                          const std::string& referenceFile, bool verbose) {
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  sdfPrt_ = std::make_shared<Sdf>("elevation");

  scalar_t thighExcess = 0.025;
  scalar_t calfExcess = 0.02;

  std::vector<std::string> collisionLinks = {"base",    "LF_thigh", "RF_thigh", "LH_thigh", "RH_thigh",
                                             "LF_calf", "RF_calf",  "LH_calf",  "RH_calf"};
  const std::vector<scalar_t>& maxExcesses = {0.05,       thighExcess, thighExcess, thighExcess, thighExcess,
                                              calfExcess, calfExcess,  calfExcess,  calfExcess};

  pinocchioSphereInterfacePrt_ = std::make_shared<PinocchioSphereInterface>(getPinocchioInterface(), collisionLinks, maxExcesses, 0.6);

  CentroidalModelPinocchioMapping pinocchioMapping(getCentroidalModelInfo());
  auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(*pinocchioSphereInterfacePrt_, pinocchioMapping);

  std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(
      new SphereSdfConstraint(*sphereKinematicsPtr, getPinocchioInterface(), pinocchioMapping, sdfPrt_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));
  getOptimalControlProblem().stateSoftConstraintPtr->add(
      "base_sdf_constraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
}

}  // namespace legged
