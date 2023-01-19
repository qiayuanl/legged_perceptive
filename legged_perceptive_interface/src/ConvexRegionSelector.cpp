//
// Created by qiayuan on 23-1-2.
//
#include "legged_perceptive_interface/ConvexRegionSelector.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <utility>

namespace legged {
ConvexRegionSelector::ConvexRegionSelector(CentroidalModelInfo info,
                                           std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                           const EndEffectorKinematics<scalar_t>& endEffectorKinematics)
    : info_(std::move(info)), planarTerrainPtr_(std::move(planarTerrainPtr)), endEffectorKinematicsPtr_(endEffectorKinematics.clone()) {}

convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
  return feetProjections_[leg][index];
}

vector3_t ConvexRegionSelector::getNominalFootholds(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
  return nominalFootholds_[leg][index];
}

void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, const vector_t& initState, TargetTrajectories& targetTrajectories) {
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto contactFlagStocks = extractContactFlags(modeSequence);
  const size_t numPhases = modeSequence.size();

  // Find start and final index of time for legs
  feet_array_t<std::vector<int>> startIndices;
  feet_array_t<std::vector<int>> finalIndices;
  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    startIndices[leg] = std::vector<int>(numPhases, 0);
    finalIndices[leg] = std::vector<int>(numPhases, 0);
    // find the startTime and finalTime indices for swing feet
    for (size_t i = 0; i < numPhases; i++) {
      // skip if it is a stance leg
      if (contactFlagStocks[leg][i]) {
        std::tie(startIndices[leg][i], finalIndices[leg][i]) = findIndex(i, contactFlagStocks[leg]);
      }
    }
  }

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    feetProjections_[leg].clear();
    nominalFootholds_[leg].clear();
    feetProjections_[leg].resize(numPhases);
    nominalFootholds_[leg].resize(numPhases);
    middleTimes_[leg].clear();

    scalar_t lastStandMiddleTime = NAN;
    // Stand leg foot
    for (size_t i = 0; i < numPhases; ++i) {
      if (contactFlagStocks[leg][i]) {
        const int standStartIndex = startIndices[leg][i];
        const int standFinalIndex = finalIndices[leg][i];
        const scalar_t standStartTime = eventTimes[standStartIndex];
        const scalar_t standFinalTime = eventTimes[standFinalIndex];
        const scalar_t standMiddleTime = standStartTime + (standFinalTime - standStartTime) / 2;

        if (!numerics::almost_eq(standMiddleTime, lastStandMiddleTime)) {
          lastStandMiddleTime = standMiddleTime;

          vector3_t nominal = getNominalFoothold(leg, standMiddleTime, initState, targetTrajectories);
          auto penaltyFunction = [](const vector3_t& /*projectedPoint*/) { return 0.0; };
          const auto projection = getBestPlanarRegionAtPositionInWorld(nominal, planarTerrainPtr_->planarRegions, penaltyFunction);

          feetProjections_[leg][i] = projection;
          nominalFootholds_[leg][i] = nominal;
          middleTimes_[leg].push_back(standMiddleTime);
        } else {
          feetProjections_[leg][i] = feetProjections_[leg][i - 1];
          nominalFootholds_[leg][i] = nominalFootholds_[leg][i - 1];
        }
      }
    }
    timeEvents_[leg] = eventTimes;
  }
}

feet_array_t<std::vector<bool>> ConvexRegionSelector::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < info_.numThreeDofContacts; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

std::pair<int, int> ConvexRegionSelector::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  if (!contactFlagStock[index]) {
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = 0;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (!contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }
  // find the final time
  int finalTimesIndex = numPhases - 2;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (!contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      break;
    }
  }
  return {startTimesIndex, finalTimesIndex};
}

vector3_t ConvexRegionSelector::getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState,
                                                   TargetTrajectories& targetTrajectories) {
  vector_t desiredState = targetTrajectories.getDesiredState(time);
  vector3_t desiredPos = centroidal_model::getBasePose(desiredState, info_).head(3);
  vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
  vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);

  auto feedback = (vector3_t() << (std::sqrt(desiredPos(2) / 9.81) * (measuredVel - desiredVel)).head(2), 0).finished();

  return endEffectorKinematicsPtr_->getPosition(targetTrajectories.getDesiredState(time))[leg] + feedback;
}

}  // namespace legged
