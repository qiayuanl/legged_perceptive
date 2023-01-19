//
// Created by qiayuan on 22-12-27.
//

#pragma once
#include "legged_perceptive_interface/visualization/SphereVisualization.h"

#include <legged_controllers/LeggedController.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class SphereSdfController : public legged::LeggedController {
 protected:
  void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file, const std::string& reference_file,
                            bool verbose) override;

  void setupMpc() override;

  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
};

}  // namespace legged
