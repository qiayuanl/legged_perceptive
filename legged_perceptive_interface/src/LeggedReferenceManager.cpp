//
// Created by qiayuan on 23-1-3.
//
#include <utility>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include "legged_perceptive_interface/LeggedReferenceManager.h"

namespace legged {

LeggedReferenceManager::LeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                               std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                               std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                               const EndEffectorKinematics<scalar_t>& endEffectorKinematics)
    : info_(std::move(info)),
      SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()) {}

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
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

contact_flag_t LeggedReferenceManager::getFootPlacementFlags(scalar_t time) const {
  contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (int i = 0; i < flag.size(); ++i) {
    flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

void LeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState, ModeSchedule& modeSchedule) {
  const size_t numPhases = modeSchedule.modeSequence.size();
  const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    auto projections = convexRegionSelectorPtr_->getProjections(leg);
    size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);
    if (contactFlagStocks[leg][initIndex]) {
      lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
      for (int i = initIndex; i < numPhases; ++i) {
        if (!contactFlagStocks[leg][i]) {
          break;
        }
        projections[i].positionInWorld = lastLiftoffPos_[leg];
      }
      for (int i = initIndex; i >= 0; --i) {
        if (!contactFlagStocks[leg][i]) {
          break;
        }
        projections[i].positionInWorld = lastLiftoffPos_[leg];
      }
    }
    if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg]) {
      for (int i = initIndex; i >= 0; --i) {
        if (contactFlagStocks[leg][i]) {
          projections[i].positionInWorld = lastLiftoffPos_[leg];
        }
        if (!contactFlagStocks[leg][i] && !contactFlagStocks[leg][i + 1]) {
          break;
        }
      }
    }

    //    for (int i = 0; i < numPhases; ++i) {
    //      if (leg == 1) std::cerr << std::setprecision(3) << projections[i].positionInWorld.z() << "\t";
    //    }
    //    std::cerr << std::endl;

    liftOffHeightSequence[leg].clear();
    liftOffHeightSequence[leg].resize(numPhases);
    touchDownHeightSequence[leg].clear();
    touchDownHeightSequence[leg].resize(numPhases);

    for (size_t i = 1; i < numPhases; ++i) {
      if (!contactFlagStocks[leg][i]) {
        liftOffHeightSequence[leg][i] =
            contactFlagStocks[leg][i - 1] ? projections[i - 1].positionInWorld.z() : liftOffHeightSequence[leg][i - 1];
      }
    }
    for (int i = numPhases - 2; i >= 0; --i) {
      if (!contactFlagStocks[leg][i]) {
        touchDownHeightSequence[leg][i] =
            contactFlagStocks[leg][i + 1] ? projections[i + 1].positionInWorld.z() : touchDownHeightSequence[leg][i + 1];
      }
    }

    //    for (int i = 0; i < numPhases; ++i) {
    //      if (leg == 1) std::cerr << std::setprecision(3) << liftOffHeightSequence[leg][i] << "\t";
    //    }
    //    std::cerr << std::endl;
    //    for (int i = 0; i < numPhases; ++i) {
    //      if (leg == 1) std::cerr << std::setprecision(3) << contactFlagStocks[leg][i] << "\t";
    //    }
    //    std::cerr << std::endl;
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

}  // namespace legged
