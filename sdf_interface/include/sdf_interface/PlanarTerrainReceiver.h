//
// Created by qiayuan on 22-12-24.
//

#pragma once

#include <mutex>

#include "Sdf.h"

#include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ros/ros.h>

namespace legged {

using namespace ocs2;

class PlanarTerrainReceiver : public SolverSynchronizedModule {
 public:
  PlanarTerrainReceiver(ros::NodeHandle nh, std::shared_ptr<Sdf> sdfPtr, const std::string& mapTopic);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void planarTerrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg);

  std::shared_ptr<Sdf> sdfPtr_;

  ros::Subscriber subscriber_;

  std::string elevationLayer_;

  std::mutex mutex_;
  std::atomic_bool updated_;

  std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
};

}  // namespace legged
