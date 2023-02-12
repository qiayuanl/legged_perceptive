cmake_minimum_required(VERSION 3.10)
project(legged_perceptive_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        controller_interface
        legged_perceptive_interface
        legged_controllers
        grid_map_ros
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        controller_interface
        legged_perceptive_interface
        legged_controllers
        grid_map_ros
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/synchronized_module/PlanarTerrainReceiver.cpp
        src/visualization/FootPlacementVisualization.cpp
        src/visualization/SphereVisualization.cpp
        src/PerceptiveController.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(grid_map_publisher
        test/MapPublisher.cpp)
target_link_libraries(grid_map_publisher
        ${catkin_LIBRARIES}
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
