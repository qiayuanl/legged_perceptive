//
// Created by qiayuan on 23-1-3.
//

#include "legged_perceptive_controllers/visualization/FootPlacementVisualization.h"
#include "legged_perceptive_controllers/visualization/SphereVisualization.h"

#include <legged_controllers/LeggedController.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class PerceptiveController : public legged::LeggedController {
 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;

  void setupMpc() override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void setupVisualization();

 private:
  std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
  std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
};

}  // namespace legged
