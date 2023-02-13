//
// Created by qiayuan on 23-1-4.
//

#pragma once
#include <ros/ros.h>

#include "legged_perceptive_interface/ConvexRegionSelector.h"
#include "legged_perceptive_interface/PerceptiveLeggedPrecomputation.h"

#include <visualization_msgs/Marker.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

namespace legged {
using namespace ocs2;

class FootPlacementVisualization {
 public:
  FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot, ros::NodeHandle& nh,
                             scalar_t maxUpdateFrequency = 20.0);

  void update(const SystemObservation& observation);

 private:
  visualization_msgs::Marker to3dRosMarker(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                           const Eigen::Isometry3d& transformPlaneToWorld, const std_msgs::Header& header, Color color,
                                           float alpha, size_t i);

  scalar_t lineWidth_ = 0.008;
  scalar_t footMarkerDiameter_ = 0.02;
  std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};

  const ConvexRegionSelector& convexRegionSelector_;

  size_t numFoot_;
  ros::Publisher markerPublisher_;
  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};
}  // namespace legged
