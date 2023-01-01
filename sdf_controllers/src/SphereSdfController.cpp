//
// Created by qiayuan on 22-12-27.
//

#include "sdf_controllers/SphereSdfController.h"
#include "sdf_interface/LeggedInterface.h"
#include "sdf_interface/PlanarTerrainReceiver.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void SphereSdfController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                               const std::string& reference_file, bool verbose) {
  leggedInterface_ = std::make_shared<SphereSdfLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
  ros::NodeHandle nh;
  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *dynamic_cast<SphereSdfLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePrt(), nh);
}

void SphereSdfController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh, dynamic_cast<SphereSdfLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<SphereSdfLeggedInterface&>(*leggedInterface_).getSdfPrt(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation_before_postprocess");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void SphereSdfController::update(const ros::Time& time, const ros::Duration& period) {
  LeggedController::update(time, period);
  sphereVisualizationPtr_->update(currentObservation_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::SphereSdfController, controller_interface::ControllerBase)
