//
// Created by qiayuan on 22-12-27.
//

#include "sdf_controllers/SphereSdfController.h"
#include "sdf_interface/GridMapReceiver.h"
#include "sdf_interface/LeggedInterface.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void SphereSdfController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                               const std::string& reference_file, bool verbose) {
  leggedInterface_ = std::make_shared<SphereSdfLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void SphereSdfController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto gridMapReceiver =
      std::make_shared<GridMapReceiver>(nh, dynamic_cast<SphereSdfLeggedInterface&>(*leggedInterface_).getSdfPrt(), "elevation_mapping");
  mpc_->getSolverPtr()->addSynchronizedModule(gridMapReceiver);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::SphereSdfController, controller_interface::ControllerBase)
