cmake_minimum_required(VERSION 2.8.3)
project(pg_driver)

add_compile_options(-std=c++11)
find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pg_driver_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libSpinnaker.so)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EXTRALIB_BIN})
