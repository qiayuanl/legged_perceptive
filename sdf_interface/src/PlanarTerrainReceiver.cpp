//
// Created by qiayuan on 22-12-24.
//

#include "sdf_interface/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <utility>

namespace legged {

PlanarTerrainReceiver::PlanarTerrainReceiver(ros::NodeHandle nh,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<grid_map::SignedDistanceField> sdfPtr, const std::string& mapTopic,
                                             std::string elevationLayer)
    : planarTerrainPtr_(std::move(planarTerrainPtr)),
      sdfPtr_(std::move(sdfPtr)),
      elevationLayer_(std::move(elevationLayer)),
      updated_(false) {
  subscriber_ = nh.subscribe(mapTopic, 1, &PlanarTerrainReceiver::planarTerrainCallback, this);
}

void PlanarTerrainReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& /*currentState*/,
                                         const ReferenceManagerInterface& /*referenceManager*/) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    updated_ = false;

    auto& elevationData = planarTerrainPtr_->gridMap.get(elevationLayer_);
    // Inpaint if needed.
    if (elevationData.hasNaN()) {
      const float inpaint{elevationData.minCoeffOfFinites()};
      ROS_WARN("[PlanarTerrainReceiver] Map contains NaN values. Will apply inpainting with min value.");
      elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
    }
    const float heightMargin{0.1};
    const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
    const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};

    *sdfPtr_ = grid_map::SignedDistanceField(planarTerrainPtr_->gridMap, elevationLayer_, minValue, maxValue);
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  *planarTerrainPtr_ = convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg));
  updated_ = true;
}

}  // namespace legged
