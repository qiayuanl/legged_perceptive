//
// Created by qiayuan on 22-12-24.
//

#pragma once

#include <mutex>

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include <ros/ros.h>

namespace legged {

using namespace ocs2;

class PlanarTerrainReceiver : public SolverSynchronizedModule {
 public:
  PlanarTerrainReceiver(ros::NodeHandle nh, std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                        std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr, const std::string& mapTopic,
                        std::string elevationLayer);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg);

  ros::Subscriber subscriber_;
  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  grid_map::SignedDistanceField signedDistanceField_;

  std::string sdfElevationLayer_;

  std::mutex mutex_;
  std::atomic_bool updated_;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
};

}  // namespace legged
