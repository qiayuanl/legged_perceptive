# legged_perceptive
## Getting Started
### Dependency

- [legged_control](https://github.com/qiayuanliao/legged_control): make sure you can run the legged_controllers in simulation or hardware;
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping): NOTE that this is ANYbotics version, which only can install from source, instead of RSL version (apt install)
- [elevation_mapping_cupy/plane_segmentation](https://github.com/leggedrobotics/elevation_mapping_cupy): NOTE that we only need 
convex_plane_decomposition_ros package (and its dependencies), DO NOT build the elevation_mapping_cupy package

### Build

    git clone git@github.com:qiayuanliao/legged_perceptive.git
    catkin build legged_perceptive_description legged_perceptive_controllers

### Run 
Launch gazbeo

    roslaunch legged_perceptive_description empty_world.launch
    
Load the controller

    roslaunch legged_perceptive_controllers load_controller.launch
