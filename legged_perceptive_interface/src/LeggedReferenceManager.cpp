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

  TargetTrajectories newTargetTrajectories;
  int nodeNum = 11;
  for (size_t i = 0; i < nodeNum; ++i) {
    scalar_t time = initTime + static_cast<double>(i) * timeHorizon / (nodeNum - 1);
    vector_t state = targetTrajectories.getDesiredState(time);
    vector_t input = targetTrajectories.getDesiredState(time);

    const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
    vector_t pos = centroidal_model::getBasePose(state, info_).head(3);
    scalar_t height = 0.4;

    // Base Orientation
    scalar_t step = 0.3;
    grid_map::Vector3 normalVector;
    normalVector(0) = (map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(step, 0))) /
                      (2 * step);
    normalVector(1) = (map.atPosition("smooth_planar", pos + grid_map::Position(0, -step)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(0, step))) /
                      (2 * step);
    normalVector(2) = 1;
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,  // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1;  // clang-format on
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

    // Base Z Position
    centroidal_model::getBasePose(state, info_)(2) =
        map.atPosition("smooth_planar", pos) + height / cos(centroidal_model::getBasePose(state, info_)(4));

    newTargetTrajectories.timeTrajectory.push_back(time);
    newTargetTrajectories.stateTrajectory.push_back(state);
    newTargetTrajectories.inputTrajectory.push_back(input);
  }
  targetTrajectories = newTargetTrajectories;

  // Footstep
  convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

  // Swing trajectory
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;
  std::tie(liftOffHeightSequence, touchDownHeightSequence) = convexRegionSelectorPtr_->getHeights();
  getSwingTrajectoryPlanner()->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

contact_flag_t LeggedReferenceManager::getFootPlacementFlags(scalar_t time) const {
  contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTime();
  for (int i = 0; i < flag.size(); ++i) {
    flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

}  // namespace legged
