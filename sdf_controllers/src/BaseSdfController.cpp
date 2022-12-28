//
// Created by qiayuan on 22-12-27.
//

#include "sdf_controllers/BaseSdfController.h"
#include "sdf_interface/GridMapReceiver.h"
#include "sdf_interface/LeggedInterface.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void BaseSdfController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file, const std::string& reference_file,
                                             bool verbose) {
  leggedInterface_ = std::make_shared<BaseSdfLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void BaseSdfController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto gridMapReceiver = std::make_shared<GridMapReceiver>(nh, dynamic_cast<BaseSdfLeggedInterface&>(*leggedInterface_).getSdfPrt(),
                                                           "elevation_mapping", "elevation");
  mpc_->getSolverPtr()->addSynchronizedModule(gridMapReceiver);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::BaseSdfController, controller_interface::ControllerBase)
