//
// Created by qiayuan on 23-1-3.
//

#include "sdf_controllers/FootPlacementController.h"
#include "sdf_interface/LeggedInterface.h"
#include "sdf_interface/synchronized_module/PlanarTerrainReceiver.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void FootPlacementController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                                   const std::string& reference_file, bool verbose) {
  leggedInterface_ = std::make_shared<FootPlacementLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void FootPlacementController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh, dynamic_cast<FootPlacementLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<FootPlacementLeggedInterface&>(*leggedInterface_).getSdfPrt(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation_before_postprocess");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::FootPlacementController, controller_interface::ControllerBase)
