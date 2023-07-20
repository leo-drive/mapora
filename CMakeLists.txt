cmake_minimum_required(VERSION 3.5)
project(mapora)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake")
set(PCL_DIR "~/libraries/libpcl-1.12.0/share/pcl-1.12/")

#find_package(PCL REQUIRED)
find_package(PCL 1.12.0 REQUIRED)
find_package(Boost 1.71 COMPONENTS filesystem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(PcapPlusPlus REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(folly REQUIRED)
find_package(gflags REQUIRED)

#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${TBB_INCLUDE_DIRS}
  ${PcapPlusPlus_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${folly_INCLUDE_DIRS}
  )

set(MAPORA_LIB_SRC
  src/mapora.cpp
  src/points_provider_velodyne.cpp
  src/continuous_packet_parser.cpp
  src/transform_provider_sbg.cpp
  src/pcl_utils.cpp
  src/slam/slam_handler.cpp
  src/slam/leo_loam/feature_extractor.cpp
  src/slam/leo_loam/init_trans_helper.cpp
  src/slam/leo_loam/init_trans_estimator.cpp
  src/slam/leo_loam/trans_estimator.cpp
  src/slam/leo_loam/mapper_occ.cpp
  src/slam/leo_loam/producer_consumer_master.cpp
  src/slam/leo_loam/slam_main.cpp
  src/utils.cpp)

set(MAPORA_LIB_HEADERS
  include/mapora/mapora.hpp
  include/mapora/date.h
  include/mapora/csv.hpp
  include/mapora/point_xyzi.hpp
  include/mapora/point_xyzit.hpp
  include/mapora/points_provider_base.hpp
  include/mapora/points_provider_velodyne.hpp
  include/mapora/continuous_packet_parser.hpp
  include/mapora/transform_provider_sbg.hpp
  include/mapora/utils.hpp
  include/mapora/time_keeper_sequential.hpp
  include/mapora/slam/slam_handler.hpp
  include/mapora/slam/leo_loam/feature_extractor.hpp
  include/mapora/slam/leo_loam/init_trans_helper.hpp
  include/mapora/slam/leo_loam/init_trans_estimator.hpp
  include/mapora/slam/leo_loam/occtree.hpp
  include/mapora/slam/leo_loam/trans_estimator.hpp
  include/mapora/slam/leo_loam/mapper_occ.hpp
  include/mapora/slam/leo_loam/producer_consumer_master.hpp
  include/mapora/slam/leo_loam/slam_main.hpp
  include/mapora/slam/leo_loam/queue_management/material_01_points_raw.hpp
  include/mapora/slam/leo_loam/queue_management/material_02_features.hpp
  include/mapora/slam/leo_loam/queue_management/material_03_init_transformations.hpp
  include/mapora/pcl_point_types.hpp
  include/mapora/visibility_control.hpp)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${MAPORA_LIB_HEADERS}
  ${MAPORA_LIB_SRC})

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  ${TBB_LIBRARIES}
  PcapPlusPlus::PcapPlusPlus
  ${GeographicLib_LIBRARIES}
  ${PCL_LIBRARIES}
  gflags
  Folly::folly
  )

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "mapora::Mapora"
  EXECUTABLE ${PROJECT_NAME}_exe)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch)