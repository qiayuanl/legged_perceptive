//
// Created by qiayuan on 22-12-30.
//

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <utility>

#include "legged_perceptive_controllers/visualization/SphereVisualization.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include <visualization_msgs/MarkerArray.h>

namespace legged {
SphereVisualization::SphereVisualization(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                         const PinocchioSphereInterface& sphereInterface, ros::NodeHandle& nh, scalar_t maxUpdateFrequency)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      centroidalModelInfo_(std::move(centroidalModelInfo)),
      sphereInterface_(sphereInterface),
      markerPublisher_(nh.advertise<visualization_msgs::MarkerArray>("sphere_markers", 1)),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

void SphereVisualization::update(const SystemObservation& observation) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    lastTime_ = observation.time;

    visualization_msgs::MarkerArray markers;

    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(observation.state, centroidalModelInfo_));

    auto positions = sphereInterface_.computeSphereCentersInWorldFrame(pinocchioInterface_);
    auto numSpheres = sphereInterface_.getNumSpheres();
    auto rads = sphereInterface_.getSphereRadii();

    int k = 0;
    for (int i = 0; i < numSpheres.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.id = i;
      marker.header.frame_id = "odom";
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.color = getColor(Color::red, 0.5);
      marker.pose.orientation = ros_msg_helpers::getOrientationMsg({1, 0, 0, 0});
      marker.scale.x = rads[k];
      marker.scale.y = marker.scale.x;
      marker.scale.z = marker.scale.x;
      for (int j = 0; j < numSpheres[i]; ++j) {
        marker.points.push_back(ros_msg_helpers::getPointMsg(positions[k + j]));
      }
      k += numSpheres[i];

      markers.markers.push_back(marker);
    }
    markerPublisher_.publish(markers);
  }
}
}  // namespace legged
