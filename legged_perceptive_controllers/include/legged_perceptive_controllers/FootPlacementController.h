//
// Created by qiayuan on 23-1-3.
//

#include "legged_perceptive_controllers/visualization/FootPlacementVisualization.h"

#include <legged_controllers/LeggedController.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class FootPlacementController : public legged::LeggedController {
 protected:
  void setupLeggedInterface(const std::string& task_file, const std::string& urdf_file, const std::string& reference_file,
                            bool verbose) override;

  void setupMpc() override;

  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
};

}  // namespace legged