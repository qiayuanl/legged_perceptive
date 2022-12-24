//
// Created by qiayuan on 22-12-24.
//

#pragma once

#include <mutex>

#include <ros/ros.h>

#include <grid_map_msgs/GridMap.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <grid_map_core/GridMap.hpp>

namespace legged {

using namespace ocs2;

class GridMapReceiver : public SolverSynchronizedModule {
 public:
  GridMapReceiver(ros::NodeHandle nh, const std::string& mapTopic, std::string elevationLayer);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void gridMapCallback(const grid_map_msgs::GridMap& msg);

  //  Grid map subscriber.
  ros::Subscriber subscriber_;

  std::string elevationLayer_;

  std::mutex mutex_;
  std::atomic_bool mapUpdated_;
  grid_map::GridMap map_;
};

}  // namespace legged