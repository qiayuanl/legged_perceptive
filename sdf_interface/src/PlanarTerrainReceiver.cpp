//
// Created by qiayuan on 22-12-24.
//

#include "sdf_interface/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace legged {

PlanarTerrainReceiver::PlanarTerrainReceiver(ros::NodeHandle nh, std::shared_ptr<Sdf> sdfPtr, const std::string& mapTopic)
    : sdfPtr_(std::move(sdfPtr)), elevationLayer_(sdfPtr_->getElevationLayer()), updated_(true) {
  planarTerrainPtr_ = std::make_unique<convex_plane_decomposition::PlanarTerrain>();
  planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
  planarTerrainPtr_->gridMap.add(elevationLayer_, 0);
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
    sdfPtr_->update(planarTerrainPtr_->gridMap);
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> newTerrain(
      new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg)));

  std::lock_guard<std::mutex> lock(mutex_);
  planarTerrainPtr_.swap(newTerrain);
  updated_ = true;
}

}  // namespace legged
