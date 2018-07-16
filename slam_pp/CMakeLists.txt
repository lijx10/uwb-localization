cmake_minimum_required(VERSION 2.8.3)
project(slam_pp)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(
    OpenCV
    NO_DEFAULT_PATH
    PATHS /usr/local/share/OpenCV /opt/ros/kinetic/share/OpenCV-3.2.0
)

#set(OpenCV_DIR "/usr/local/share/OpenCV")
#set(OpenCV_DIR "/opt/ros/kinetic/share/OpenCV-3.2.0")
set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common_msgs REQUIRED

  visualization_msgs
  nav_msgs

  cv_bridge
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
  pcl_conversions
  pcl_msgs
  pcl_ros
)

# pay attention that Pangolin should be modified and compiled with boost and c++99.
find_package(PCL 1.7 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_library(Reflexxes_LIBRARIES flexxes ${PROJECT_SOURCE_DIR}/exLib/Reflexxes)
set(Relfexxes_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/exLib/Reflexxes/Reflexxes)

find_package(Boost REQUIRED COMPONENTS system thread iostreams chrono)

find_package(Eigen3)
find_package(Ceres REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS common_msgs roscpp rospy std_msgs tf pcl_conversions
  DEPENDS PCL
)

###########
## Build ##
###########

#MESSAGE(STATUS "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
#MESSAGE(STATUS ${CMAKE_CXX_COMPILER})
#MESSAGE(STATUS ${OpenCV_LIBRARIES})
#MESSAGE(STATUS ${OpenCV_INCLUDE_DIRS})
#MESSAGE(STATUS ${dbow2_lib})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Relfexxes_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/exLib
  ${Eigen_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


# with C++99, please use ORB_SLAM2_boost
set(Library_Set
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Reflexxes_LIBRARIES}
    ${CERES_LIBRARIES}
)



## Declare a C++ library
file(GLOB_RECURSE HEADERS */*.hpp */*.h)

## Declare a C++ executable
add_executable(slam_pp_node
    ${HEADERS} # for qtcreator...

    src/basic_function.cpp
    src/slam_pp_node.cpp

    src/slam/slam_abstract.cpp
    src/slam/thread_slam.cpp
    src/slam/uwb_localization.cpp
    src/slam/uwb_node.cpp
    src/slam/uwb_loc_init.cpp

    src/mapping/mapping_abstract.cpp
    src/mapping/thread_mapping.cpp

    src/pp/pp_abstract.cpp
    src/pp/simple_planner.cpp
    src/pp/thread_pp.cpp

)

## Specify libraries to link a library or executable target against

target_link_libraries(slam_pp_node
    ${Library_Set}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(slam_pp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
