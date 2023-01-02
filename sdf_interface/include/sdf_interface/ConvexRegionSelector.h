//
// Created by qiayuan on 23-1-2.
//

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class ConvexRegionSelector {
 public:
  struct Config {
    size_t numberOfVertices = 16;
    scalar_t growthFactor = 1.05;
  };
  ConvexRegionSelector(Config config, size_t numFeet, std::shared_ptr<convex_plane_decomposition::PlanarTerrain> PlanarTerrainPtr);

  void update(const ModeSchedule& modeSchedule, const vector_t& initState, TargetTrajectories& targetTrajectories);

 private:
  feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
  feet_array_t<std::vector<convex_plane_decomposition::CgalPolygon2d>> feetConvexRegions_;
  feet_array_t<std::vector<scalar_t>> feetConvexRegionEvents_;

  const Config config_;
  const size_t numFeet_;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr_;
};
}  // namespace legged
