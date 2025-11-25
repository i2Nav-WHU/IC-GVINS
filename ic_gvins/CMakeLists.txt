cmake_minimum_required(VERSION 3.10)
project(ic_gvins)

set(CMAKE_CXX_STANDARD 14)

# Optimization
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(Ceres_DIR "$ENV{HOME}/ceres-2.1/lib/cmake/Ceres")

# Catkin
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        rospy 
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        message_filters
)

catkin_package(
        LIBRARIES ic_gvins_core
        CATKIN_DEPENDS 
            roscpp
            rospy
            std_msgs 
            sensor_msgs 
            cv_bridge
            image_transport
            message_filters
)

# Build GVINS library
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ic_gvins
        ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE GVINS_SOURCE
        ic_gvins/*.cc
        ic_gvins/common/*.cc
        ic_gvins/fileio/*.cc
        ic_gvins/tracking/*.cc
        ic_gvins/preintegration/*.cc)
        
add_library(ic_gvins_core SHARED
        ${GVINS_SOURCE})

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(ic_gvins_core ${OpenCV_LIBS})

# glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
target_link_libraries(ic_gvins_core ${GLOG_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(ic_gvins_core ${YAML_CPP_LIBRARIES})

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(ic_gvins_core ${CERES_LIBRARIES})

# boost
find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(ic_gvins_core ${Boost_LIBRARIES})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(ic_gvins_core TBB::tbb)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(thirdparty/abseil-cpp abseil-cpp)
target_link_libraries(ic_gvins_core
        absl::strings
        absl::str_format
        absl::time)

# Catkin libraries
target_link_libraries(ic_gvins_core ${catkin_LIBRARIES})

# GVINS ROS node
add_executable(${PROJECT_NAME}_ros
        ROS/fusion_ros.cc
        ROS/drawer_rviz.cc)

add_dependencies(${PROJECT_NAME}_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ros
        ${catkin_LIBRARIES}
        ic_gvins_core)

# ============================================
# Python Scripts Installation
# ============================================
catkin_install_python(PROGRAMS
  scripts/yolo_mask_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ============================================
# Install Resources
# ============================================
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
  FILES_MATCHING PATTERN "*"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)