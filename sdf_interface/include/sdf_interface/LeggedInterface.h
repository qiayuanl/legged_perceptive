//
// Created by qiayuan on 22-12-27.
//

#pragma once

#include "sdf_interface/SphereSdfConstraint.h"

#include <legged_interface/LeggedInterface.h>

namespace legged {

class SphereSdfLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  std::shared_ptr<Sdf> getSdfPrt() const { return sdfPrt_; }

  std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePrt() const { return pinocchioSphereInterfacePrt_; }

 private:
  std::shared_ptr<Sdf> sdfPrt_;
  std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePrt_;
};

}  // namespace legged