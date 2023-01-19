//
// Created by qiayuan on 23-1-3.
//
#pragma once

#include <memory>

#include "legged_perceptive_interface/ConvexRegionSelector.h"

#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class LeggedReferenceManager : public SwitchedModelReferenceManager {
 public:
  LeggedReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                         std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr);

  const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

 private:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
};

}  // namespace legged
