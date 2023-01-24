//
// Created by qiayuan on 22-12-24.
//

#include "legged_perceptive_interface/synchronized_module/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <utility>

namespace legged {

PlanarTerrainReceiver::PlanarTerrainReceiver(ros::NodeHandle nh,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr,
                                             const std::string& mapTopic, std::string elevationLayer)
    : signedDistanceField_(*signedDistanceFieldPtr),
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      sdfPtr_(std::move(signedDistanceFieldPtr)),
      elevationLayer_(std::move(elevationLayer)),
      updated_(false) {
  subscriber_ = nh.subscribe(mapTopic, 1, &PlanarTerrainReceiver::planarTerrainCallback, this);
}

void PlanarTerrainReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& /*currentState*/,
                                         const ReferenceManagerInterface& /*referenceManager*/) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    updated_ = false;

    *planarTerrainPtr_ = planarTerrain_;
    *sdfPtr_ = signedDistanceField_;
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  updated_ = true;

  planarTerrain_ = convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg));

  auto& elevationData = planarTerrainPtr_->gridMap.get(elevationLayer_);
  if (elevationData.hasNaN()) {
    const float inpaint{elevationData.minCoeffOfFinites()};
    ROS_WARN("[PlanarTerrainReceiver] Map contains NaN values. Will apply inpainting with min value.");
    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
  }
  const float heightMargin{0.1};
  const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
  const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};
  signedDistanceField_ = grid_map::SignedDistanceField(planarTerrainPtr_->gridMap, elevationLayer_, minValue, maxValue);
}

}  // namespace legged
