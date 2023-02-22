//
// Created by qiayuan on 23-1-3.
//

#include <pinocchio/fwd.hpp>

#include "legged_perceptive_controllers/PerceptiveController.h"

#include "legged_perceptive_controllers/synchronized_module/PlanarTerrainReceiver.h"
#include "legged_perceptive_interface/PerceptiveLeggedInterface.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void PerceptiveController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                bool verbose) {
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  setupVisualization();
}

void PerceptiveController::setupVisualization() {
  ros::NodeHandle nh;
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
      leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, nh);

  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePtr(), nh);
}

void PerceptiveController::setupMpc() {
  LeggedController::setupMpc();

  ros::NodeHandle nh;
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh, dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void PerceptiveController::update(const ros::Time& time, const ros::Duration& period) {
  LeggedController::update(time, period);
  footPlacementVisualizationPtr_->update(currentObservation_);
  sphereVisualizationPtr_->update(currentObservation_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveController, controller_interface::ControllerBase)
