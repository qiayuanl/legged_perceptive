//
// Created by qiayuan on 23-1-2.
//
#include "sdf_interface/ConvexRegionSelector.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>

#include <utility>

namespace legged {
ConvexRegionSelector::ConvexRegionSelector(ConvexRegionSelector::Config config, size_t numFeet,
                                           std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr)
    : config_(config), numFeet_(numFeet), planarTerrainPtr_(std::move(planarTerrainPtr)) {}

void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, const vector_t& initState, TargetTrajectories& targetTrajectories) {
  vector3_t query{0, 0, 0};
  auto penaltyFunction = [](const vector3_t& /*projectedPoint*/) { return 0.0; };

  const auto projection = getBestPlanarRegionAtPositionInWorld(query, planarTerrainPtr_->planarRegions, penaltyFunction);

  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
      projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, config_.numberOfVertices, config_.growthFactor);

  projection.regionPtr->transformPlaneToWorld;
}

}  // namespace legged
