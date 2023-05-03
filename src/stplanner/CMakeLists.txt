cmake_minimum_required(VERSION 3.0.2)
project(stplanner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_filters
)

find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calc_path_length_and_map_between_point_and_s.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/calc_speed_planning_start_condition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/generate_convex_space.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/generate_st_graph.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/increase_st_point_count.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/interp1.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/osqp-eigen.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/path_and_speed_merge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/speed_decision_with_dynamic_planning.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/speed_planning_with_quadratic_programming.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stitch.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/stplanner.cpp
)

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/main.h
)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stplanner src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(stplanner ${catkin_LIBRARIES})
target_link_libraries(stplanner OsqpEigen::OsqpEigen ${catkin_LIBRARIES})
# target_link_libraries(stplanner PRIVATE osqp::osqpstatic)

