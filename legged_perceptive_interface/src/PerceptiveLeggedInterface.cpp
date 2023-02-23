//
// Created by qiayuan on 22-12-27.
//

#include "legged_perceptive_interface/constraint/FootCollisionConstraint.h"
#include "legged_perceptive_interface/constraint/SphereSdfConstraint.h"

#include "legged_perceptive_interface/ConvexRegionSelector.h"
#include "legged_perceptive_interface/PerceptiveLeggedInterface.h"
#include "legged_perceptive_interface/PerceptiveLeggedPrecomputation.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <memory>

namespace legged {

void PerceptiveLeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool verbose) {
  planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

  double width{5.0}, height{5.0};
  convex_plane_decomposition::PlanarRegion plannerRegion;
  plannerRegion.transformPlaneToWorld.setIdentity();
  plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2, -width / 2, +height / 2, width / 2);
  convex_plane_decomposition::CgalPolygonWithHoles2d boundary;
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, +width / 2));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, +width / 2));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, -width / 2));
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, -width / 2));
  plannerRegion.boundaryWithInset.boundary = boundary;
  convex_plane_decomposition::CgalPolygonWithHoles2d insets;
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, +width / 2 - 0.01));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, +width / 2 - 0.01));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.01, -width / 2 + 0.01));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.01, -width / 2 + 0.01));
  plannerRegion.boundaryWithInset.insets.push_back(insets);
  planarTerrainPtr_->planarRegions.push_back(plannerRegion);

  std::string layer = "elevation_before_postprocess";
  planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);
  planarTerrainPtr_->gridMap.add(layer, 0);
  planarTerrainPtr_->gridMap.add("smooth_planar", 0);
  signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>(planarTerrainPtr_->gridMap, layer, -0.1, 0.1);

  LeggedInterface::setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings().contactNames3DoF[i];
    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

    std::unique_ptr<PenaltyBase> placementPenalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-4)));
    std::unique_ptr<PenaltyBase> collisionPenalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-3)));

    // For foot placement
    std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
        new FootPlacementConstraint(*referenceManagerPtr_, *eeKinematicsPtr, i, numVertices_));
    problemPtr_->stateSoftConstraintPtr->add(
        footName + "_footPlacement",
        std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footPlacementConstraint), std::move(placementPenalty))));

    // For foot Collision
    std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
        new FootCollisionConstraint(*referenceManagerPtr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i, 0.03));
    problemPtr_->stateSoftConstraintPtr->add(
        footName + "_footCollision",
        std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footCollisionConstraint), std::move(collisionPenalty))));
  }

  // For collision avoidance
  scalar_t thighExcess = 0.025;
  scalar_t calfExcess = 0.02;

  std::vector<std::string> collisionLinks = {"LF_calf", "RF_calf", "LH_calf", "RH_calf"};
  const std::vector<scalar_t>& maxExcesses = {calfExcess, calfExcess, calfExcess, calfExcess};

  pinocchioSphereInterfacePtr_ = std::make_shared<PinocchioSphereInterface>(*pinocchioInterfacePtr_, collisionLinks, maxExcesses, 0.6);

  CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
  auto sphereKinematicsPtr = std::make_unique<PinocchioSphereKinematics>(*pinocchioSphereInterfacePtr_, pinocchioMapping);

  std::unique_ptr<SphereSdfConstraint> sphereSdfConstraint(new SphereSdfConstraint(*sphereKinematicsPtr, signedDistanceFieldPtr_));

  //  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-3, 1e-3)));
  //  problemPtr_->stateSoftConstraintPtr->add(
  //      "sdfConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(sphereSdfConstraint), std::move(penalty))));
}

void PerceptiveLeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& /*urdfFile*/,
                                                      const std::string& referenceFile, bool verbose) {
  auto swingTrajectoryPlanner =
      std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({modelSettings_.contactNames3DoF}, "ALL_FOOT");
  auto convexRegionSelector =
      std::make_unique<ConvexRegionSelector>(centroidalModelInfo_, planarTerrainPtr_, *eeKinematicsPtr, numVertices_);

  scalar_t comHeight = 0;
  loadData::loadCppDataType(referenceFile, "comHeight", comHeight);
  referenceManagerPtr_.reset(new PerceptiveLeggedReferenceManager(centroidalModelInfo_, loadGaitSchedule(referenceFile, verbose),
                                                                  std::move(swingTrajectoryPlanner), std::move(convexRegionSelector),
                                                                  *eeKinematicsPtr, comHeight));
}

void PerceptiveLeggedInterface::setupPreComputation(const std::string& /*taskFile*/, const std::string& /*urdfFile*/,
                                                    const std::string& /*referenceFile*/, bool /*verbose*/) {
  problemPtr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_,
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_).getConvexRegionSelectorPtr());
}

}  // namespace legged
