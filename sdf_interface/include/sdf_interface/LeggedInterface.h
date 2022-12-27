//
// Created by qiayuan on 22-12-27.
//

#pragma once

#include "sdf_interface/BaseSdfConstraint.h"

#include <legged_interface/LeggedInterface.h>

namespace legged {

class BaseSdfLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  std::shared_ptr<grid_map::SignedDistanceField> getSdfPrt() const { return sdfPrt_; }

 private:
  std::shared_ptr<grid_map::SignedDistanceField> sdfPrt_;
};

}  // namespace legged
