cmake_minimum_required(VERSION 2.8.3)
project(wipe_table)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)

# Libraries


cs_add_executable(wipe_table src/wipe_table.cpp)

target_link_libraries(wipe_table cart_path_planner_lib cwru_pcl_utils ${catkin_LIBRARIES})

cs_install()
cs_export()
