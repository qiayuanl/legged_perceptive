//
// Created by qiayuan on 22-12-27.
//

#include "sdf_interface/LeggedInterface.h"
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

namespace legged {
void BaseSdfLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                        const std::string& referenceFile, bool verbose) {
  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));

  std::unique_ptr<BaseSdfConstraint> baseSdfConstraint(new BaseSdfConstraint(sdfPrt_, 0.15, getCentroidalModelInfo()));
  getOptimalControlProblem().stateSoftConstraintPtr->add(
      "base_sdf_constraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(baseSdfConstraint), std::move(penalty))));
}

}  // namespace legged
