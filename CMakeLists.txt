cmake_minimum_required(VERSION 3.5)
project(mapora)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost 1.74 COMPONENTS filesystem
        serialization iostreams system thread program_options REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(libLAS REQUIRED)
find_package(PcapPlusPlus REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(TBB REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(include
        ${Boost_INCLUDE_DIR}
        ${TBB_INCLUDE_DIRS}
        ${PcapPlusPlus_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
        ${libLAS_INCLUDE_DIRS})

set(MAPORA_LIB_SRC
        src/mapora.cpp
        src/mapora_rosbag.cpp
        src/points_provider_velodyne_vlp16.cpp
        src/parsers/velodyne_vlp16.cpp
        src/parsers/hesai_xt32.cpp
        src/transform_provider_applanix.cpp
        src/utils.cpp)

set(MAPORA_LIB_HEADERS
        include/mapora/mapora.hpp
        include/mapora/mapora_rosbag.hpp
        include/mapora/date.h
        include/mapora/csv.hpp
        include/mapora/point_types.hpp
        include/mapora/points_provider_velodyne_vlp16.hpp
        include/mapora/parsers/velodyne_vlp16.hpp
        include/mapora/parsers/hesai_xt32.hpp
        include/mapora/transform_provider_applanix.hpp
        include/mapora/utils.hpp
        include/mapora/visibility_control.hpp)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        ${MAPORA_LIB_HEADERS}
        ${MAPORA_LIB_SRC})
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
#        ${PcapPlusPlus_LIBRARIES}
        Eigen3::Eigen
         PcapPlusPlus::PcapPlusPlus
        TBB::tbb
        ${GeographicLib_LIBRARIES}
        ${libLAS_LIBRARIES})

ament_auto_add_library(mapora_rosbag SHARED
        ${MAPORA_LIB_HEADERS}
        ${MAPORA_LIB_SRC})
target_link_libraries(mapora_rosbag
        ${Boost_LIBRARIES}
        Eigen3::Eigen
        TBB::tbb
        ${GeographicLib_LIBRARIES}
        ${libLAS_LIBRARIES})

#target_compile_options(${PROJECT_NAME} PRIVATE
#  -Wall
#  -Werror
#  -Wextra
#  #-Wshadow             # causes issues with ROS 2 headers
#  #-Wnon-virtual-dtor   # causes issues with ROS 2 headers
#  -pedantic
#  -Wcast-align
#  -Wunused
#  -Wconversion
#  -Wsign-conversion
#  -Wdouble-promotion
#  #-Wnull-dereference    # gcc6
#  #-Wduplicated-branches # gcc7
#  #-Wduplicated-cond     # gcc6
#  #-Wrestrict            # gcc7
#  -fvisibility=hidden)
# C++-only options

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN "mapora::Mapora"
        EXECUTABLE ${PROJECT_NAME}_exe)
rclcpp_components_register_node(mapora_rosbag
        PLUGIN "mapora::MaporaRosbag"
        EXECUTABLE mapora_rosbag_exe)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
        params
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries({${PROJECT_NAME}_node})

ament_auto_package()
