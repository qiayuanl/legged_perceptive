//
// Created by qiayuan on 23-2-4.
//

#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

int main(int argc, char** argv) {
  // Initialize node and publisher.
  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle nh("~");
  ros::Publisher gridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("/elevation_mapping/elevation_map_raw", 1, true);

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener transformListener(buffer);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("odom");
  map.setGeometry(Length(10.0, 10.0), 0.03);
  map["elevation"].setConstant(0);

  grid_map::Polygon polygon;
  polygon.setFrameId(map.getFrameId());
  polygon.addVertex(Position(0.5, 0.3));
  polygon.addVertex(Position(0.6, 0.3));
  polygon.addVertex(Position(0.6, -0.3));
  polygon.addVertex(Position(0.5, -0.3));

  for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) {
    map.at("elevation", *iterator) = 0.16;
  }

  ros::Rate rate(3.0);
  while (nh.ok()) {
    // Add data to grid map.
    ros::Time time = ros::Time::now();

    try {
      geometry_msgs::TransformStamped tran = buffer.lookupTransform("odom", "base", ros::Time(0));
      map.move(Position(tran.transform.translation.x, tran.transform.translation.y));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("Failure %s\n", ex.what());
    }

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    gridMapPublisher.publish(message);

    rate.sleep();
  }

  return 0;
}
