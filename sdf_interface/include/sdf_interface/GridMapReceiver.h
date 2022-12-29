//
// Created by qiayuan on 22-12-24.
//

#pragma once

#include <mutex>

#include "Sdf.h"

#include <grid_map_msgs/GridMap.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ros/ros.h>

namespace legged {

using namespace ocs2;

class GridMapReceiver : public SolverSynchronizedModule {
 public:
  GridMapReceiver(ros::NodeHandle nh, std::shared_ptr<Sdf> sdfPtr, const std::string& mapTopic);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void gridMapCallback(const grid_map_msgs::GridMap& msg);

  std::shared_ptr<Sdf> sdfPtr_;

  ros::Subscriber subscriber_;

  std::string elevationLayer_;

  std::mutex mutex_;
  std::atomic_bool mapUpdated_;
  grid_map::GridMap map_;
};

}  // namespace legged