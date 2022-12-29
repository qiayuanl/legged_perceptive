//
// Created by qiayuan on 22-12-29.
//

#pragma once

#include <memory>

#include <grid_map_sdf/SignedDistanceField.hpp>

namespace legged {

class Sdf {
 public:
  explicit Sdf(std::string elevationLayer) : elevationLayer_(std::move(elevationLayer)){};

  void update(grid_map::GridMap& map) {
    auto& elevationData = map.get(elevationLayer_);

    const float heightMargin{0.1};
    const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
    const float maxValue{elevationData.maxCoeffOfFinites() + heightMargin};
    sdfPtr_ = std::make_shared<grid_map::SignedDistanceField>(map, elevationLayer_, minValue, maxValue);
  }

  double value(const grid_map::Position3& position) const { return sdfPtr_->value(position); }

  grid_map::SignedDistanceField::Derivative3 derivative(const grid_map::Position3& position) const {
    return sdfPtr_->derivative(position);
  };

  std::string getElevationLayer() const { return elevationLayer_; }

 private:
  std::string elevationLayer_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
};

}  // namespace legged