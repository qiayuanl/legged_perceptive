//
// Created by qiayuan on 22-12-30.
//

#include "legged_perceptive_controllers/visualization/FootPlacementVisualization.h"

#include <visualization_msgs/MarkerArray.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace legged {
FootPlacementVisualization::FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot, size_t numVertices,
                                                       ros::NodeHandle& nh, scalar_t maxUpdateFrequency)
    : convexRegionSelector_(convexRegionSelector),
      numFoot_(numFoot),
      numVertices_(numVertices),
      markerPublisher_(nh.advertise<visualization_msgs::MarkerArray>("foot_placement", 1)),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

void FootPlacementVisualization::update(const SystemObservation& observation) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    lastTime_ = observation.time;

    std_msgs::Header header;
    //    header.stamp.fromNSec(planarTerrainPtr->gridMap.getTimestamp());
    header.frame_id = "odom";

    visualization_msgs::MarkerArray makerArray;

    size_t i = 0;
    for (int leg = 0; leg < numFoot_; ++leg) {
      auto middleTimes = convexRegionSelector_.getMiddleTimes(leg);

      for (int k = 1; k < middleTimes.size(); ++k) {
        if (observation.time < middleTimes[k - 1] || observation.time > middleTimes[k]) {
          continue;
        }
        const auto projection = convexRegionSelector_.getProjection(leg, middleTimes[k]);
        if (projection.regionPtr == nullptr) {
          continue;
        }
        auto color = feetColorMap_[leg];

        // Projections
        auto projectionMaker = getArrowAtPointMsg(projection.regionPtr->transformPlaneToWorld.linear() * vector3_t(0, 0, 0.1),
                                                  projection.positionInWorld, color);
        projectionMaker.header = header;
        projectionMaker.ns = "Projections";
        projectionMaker.id = i;
        makerArray.markers.push_back(projectionMaker);

        // Convex Region
        double growthFactor = 1.05;
        const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
            projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor);
        auto convexRegionMsg =
            convex_plane_decomposition::to3dRosPolygon(convexRegion, projection.regionPtr->transformPlaneToWorld, header);
        makerArray.markers.push_back(to3dRosMarker(convexRegion, projection.regionPtr->transformPlaneToWorld, header, color, i));

        // Nominal Footholds
        const auto nominal = convexRegionSelector_.getNominalFootholds(leg, middleTimes[k]);
        auto nominalMarker = getFootMarker(nominal, true, color, footMarkerDiameter_, 1.);
        nominalMarker.header = header;
        nominalMarker.ns = "Nominal Footholds";
        nominalMarker.id = i;
        makerArray.markers.push_back(nominalMarker);

        i++;
      }
    }

    markerPublisher_.publish(makerArray);
  }
}

visualization_msgs::Marker FootPlacementVisualization::to3dRosMarker(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                                                     const Eigen::Isometry3d& transformPlaneToWorld,
                                                                     const std_msgs::Header& header, Color color, size_t i) {
  visualization_msgs::Marker marker;
  marker.ns = "Convex Regions";
  marker.id = i;
  marker.header = header;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = lineWidth_;
  marker.color = getColor(color);
  if (!polygon.is_empty()) {
    marker.points.reserve(polygon.size() + 1);
    for (const auto& point : polygon) {
      const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(point, transformPlaneToWorld);
      geometry_msgs::Point point_ros;
      point_ros.x = pointInWorld.x();
      point_ros.y = pointInWorld.y();
      point_ros.z = pointInWorld.z();
      marker.points.push_back(point_ros);
    }
    // repeat the first point to close to polygon
    const auto pointInWorld =
        convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(polygon.vertex(0), transformPlaneToWorld);
    geometry_msgs::Point point_ros;
    point_ros.x = pointInWorld.x();
    point_ros.y = pointInWorld.y();
    point_ros.z = pointInWorld.z();
    marker.points.push_back(point_ros);
  }
  marker.pose.orientation.w = 1.0;
  return marker;
}

}  // namespace legged
