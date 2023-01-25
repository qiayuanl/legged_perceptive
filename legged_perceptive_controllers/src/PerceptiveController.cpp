//
// Created by qiayuan on 23-1-3.
//

#include "legged_perceptive_controllers/PerceptiveController.h"
#include "legged_perceptive_interface/PerceptiveLeggedInterface.h"
#include "legged_perceptive_interface/synchronized_module/LeggedReferenceManager.h"
#include "legged_perceptive_interface/synchronized_module/PlanarTerrainReceiver.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void PerceptiveController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                                const std::string& reference_file, bool verbose) {
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(task_file, urdf_file, reference_file, verbose);
  leggedInterface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);

  ros::NodeHandle nh;
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
      *dynamic_cast<LeggedReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
      leggedInterface_->getCentroidalModelInfo().numThreeDofContacts,
      dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getNumVertices(), nh);

  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePrt(), nh);
}

void PerceptiveController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh, dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation_before_postprocess");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void PerceptiveController::update(const ros::Time& time, const ros::Duration& period) {
  LeggedController::update(time, period);
  footPlacementVisualizationPtr_->update(currentObservation_);
  sphereVisualizationPtr_->update(currentObservation_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveController, controller_interface::ControllerBase)
