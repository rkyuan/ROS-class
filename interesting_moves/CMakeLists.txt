cmake_minimum_required(VERSION 2.8.3)
project(interesting_moves)

find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
cs_add_library(interesting_moves src/interestingMoves.cpp)   

# Executables
cs_add_executable(interestingMoves src/interestingMoves.cpp)
target_link_libraries(interestingMoves interesting_moves ${catkin_LIBRARIES})

cs_install()
cs_export()
