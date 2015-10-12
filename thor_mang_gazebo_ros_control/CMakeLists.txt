cmake_minimum_required(VERSION 2.8.3)
project(thor_mang_gazebo_ros_control)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_manager
  gazebo_ros_control
  hector_gazebo_plugins
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    controller_manager
    gazebo_ros_control
    hector_gazebo_plugins
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} thor_mang_robot_hw_sim
  DEPENDS gazebo
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Libraries
#add_library(${PROJECT_NAME} src/thor_mang_gazebo_ros_control_plugin.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(thor_mang_robot_hw_sim src/thor_mang_robot_hw_sim.cpp)
target_link_libraries(thor_mang_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Install
install(TARGETS thor_mang_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES thor_mang_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
