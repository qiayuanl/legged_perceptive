//
// Created by qiayuan on 23-1-3.
//
#pragma once

#include <memory>

#include "legged_perceptive_interface/ConvexRegionSelector.h"

#include <legged_interface/SwitchedModelReferenceManager.h>

namespace legged {

using namespace ocs2;
using namespace legged_robot;

class LeggedReferenceManager : public SwitchedModelReferenceManager {
 public:
  LeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                         std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                         std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics);

  const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() { return convexRegionSelectorPtr_; }

  contact_flag_t getFootPlacementFlags(scalar_t time) const;

 protected:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;

  void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState, ModeSchedule& modeSchedule);

  void modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                         const feet_array_t<std::vector<bool>>& contactFlagStocks,
                         std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);

  const CentroidalModelInfo info_;
  feet_array_t<vector3_t> lastLiftoffPos_;

  std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
};

}  // namespace legged
