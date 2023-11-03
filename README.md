# legged_perceptive
[![](https://i.ytimg.com/vi/zcTuBe6d1qQ/maxresdefault.jpg)](https://youtu.be/zcTuBe6d1qQ?si=yn4POSM8wvb2f0C8)

## Getting Started
### Dependency

- [legged_control](https://github.com/qiayuanliao/legged_control): make sure you can run the legged_controllers in simulation or hardware;
- [grid_map](https://github.com/ANYbotics/grid_map): NOTE that this is ANYbotics version, which only can install from source, instead of RSL version (apt install);
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping);
- [elevation_mapping_cupy/plane_segmentation](https://github.com/leggedrobotics/elevation_mapping_cupy): NOTE that we only need 
convex_plane_decomposition_ros package (and its dependencies), DO NOT build the elevation_mapping_cupy package
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense2_description](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development/realsense2_description)


### Build

    git clone git@github.com:qiayuanliao/legged_perceptive.git
    catkin build legged_perceptive_description legged_perceptive_controllers

### Run 
Launch gazbeo

    roslaunch legged_perceptive_description empty_world.launch
    
Load the controller

    roslaunch legged_perceptive_controllers load_controller.launch
