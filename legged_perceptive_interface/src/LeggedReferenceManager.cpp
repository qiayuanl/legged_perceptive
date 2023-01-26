//
// Created by qiayuan on 23-1-3.
//
#include <utility>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "legged_perceptive_interface/LeggedReferenceManager.h"

namespace legged {

LeggedReferenceManager::LeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                               std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                               std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr)
    : info_(std::move(info)),
      SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)) {}

void LeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                              TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  const scalar_t terrainHeight = 0.0;
  getSwingTrajectoryPlanner()->update(modeSchedule, terrainHeight);

  convexRegionSelectorPtr_->update(modeSchedule, initState, targetTrajectories);

  for (size_t i = 0; i < targetTrajectories.size(); ++i) {
    vector_t pos = centroidal_model::getBasePose(targetTrajectories.stateTrajectory[i], info_).head(3);
    centroidal_model::getBasePose(targetTrajectories.stateTrajectory[i], info_)(2) +=
        convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap.atPosition("smooth_planar", pos);
  }
}

}  // namespace legged
