//
// Created by qiayuan on 23-1-3.
//
#include <utility>

#include "legged_perceptive_interface/LeggedReferenceManager.h"

namespace legged {

LeggedReferenceManager::LeggedReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                               std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                               std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr)
    : SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)) {}

void LeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                              TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const scalar_t terrainHeight = 0.0;
  getSwingTrajectoryPlanner()->update(modeSchedule, terrainHeight);

  convexRegionSelectorPtr_->update(modeSchedule, initState, targetTrajectories);
}

}  // namespace legged
