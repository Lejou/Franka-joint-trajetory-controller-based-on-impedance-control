cmake_minimum_required(VERSION 3.0.2)
project(franka_effort_controller)

SET(CMAKE_BUILD_TYPE release)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_gripper
  franka_hw
  geometry_msgs
  hardware_interface
  joint_limits_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  tf
  tf_conversions
  trajectory_msgs
  urdf
  visualization_msgs
  std_msgs
)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)

find_package(SpaceVecAlg REQUIRED)
find_package(RBDyn REQUIRED)
# find_package(mc_rbdyn_urdf REQUIRED)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/CarImpParam.cfg
  cfg/ComplianceParam.cfg
  cfg/Wrench.cfg

)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    angles
    control_msgs
    control_toolbox
    controller_interface
    dynamic_reconfigure
    hardware_interface
    geometry_msgs
    eigen_conversions
    joint_limits_interface
    realtime_tools
    roscpp
    rospy
    franka_hw
    franka_gripper
    trajectory_msgs
    urdf
    tf
    tf_conversions
    message_runtime
    pluginlib
    visualization_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  DEPENDS
    Boost
    Franka
    # mc_rbdyn_urdf
)



include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
                            src/franka_compliance_controller.cpp
                            src/urdf.cpp
                            src/franka_force_controller.cpp
                            src/franka_car_imp_controller.cpp
                            # include/franka_effort_controller/franka_force_controller.h
                            # include/franka_effort_controller/franka_compliance_controller.h
                            # include/franka_effort_controller/Frhardware_interface_adapter.h
                            # include/franka_effort_controller/rbdyn_wrapper.h
                            # include/franka_effort_controller/urdf.h
                            # include/franka_effort_controller/pseudo_inversion.h
)

catkin_install_python(
  PROGRAMS 
    scripts/data_record.py
    scripts/sg_exp_with_phantom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} RBDyn mc_rbdyn_urdf)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} RBDyn)

# add_executable(franka_effort_controller_node
#   src/franka_effort_controller.cpp
#   include/franka_effort_controller/franka_effort_controller.h
#   include/franka_effort_controller/franka_effort_controller_imp.h
# )

# add_dependencies(franka_effort_controller_node
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(franka_effort_controller_node
#   ${Franka_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

# target_include_directories(franka_effort_controller_node SYSTEM PUBLIC
#   ${Franka_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
# )

## Installation
# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
# Install plugins
install(FILES franka_ros_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
