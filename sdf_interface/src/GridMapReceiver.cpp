//
// Created by qiayuan on 22-12-24.
//

#include "sdf_interface/GridMapReceiver.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <utility>

namespace legged {
GridMapReceiver::GridMapReceiver(ros::NodeHandle nh, std::shared_ptr<grid_map::SignedDistanceField> sdfPtr, const std::string& mapTopic,
                                 std::string elevationLayer)
    : sdfPtr_(std::move(sdfPtr)), elevationLayer_(std::move(elevationLayer)), mapUpdated_(false) {
  subscriber_ = nh.subscribe(mapTopic, 1, &GridMapReceiver::gridMapCallback, this);
}

void GridMapReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& /*currentState*/,
                                   const ReferenceManagerInterface& /*referenceManager*/) {
  if (mapUpdated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    mapUpdated_ = false;

    auto& elevationData = map_.get(elevationLayer_);
    // Inpaint if needed.
    if (elevationData.hasNaN()) {
      const float inpaint{elevationData.minCoeffOfFinites()};
      ROS_WARN("[GridMapReceiver] Map contains NaN values. Will apply inpainting with min value.");
      elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
    }

    // Generate SDF.
    const float heightMargin{0.1};
    const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
    const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};
    sdfPtr_ = std::make_shared<grid_map::SignedDistanceField>(map_, elevationLayer_, minValue, maxValue);
    sdfPtr_->value(grid_map::Position3(0, 0, 0));
  }
}

void GridMapReceiver::gridMapCallback(const grid_map_msgs::GridMap& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Convert message to map
  std::vector<std::string> layers{elevationLayer_};
  grid_map::GridMapRosConverter::fromMessage(msg, map_, layers, false, false);
}

}  // namespace legged
