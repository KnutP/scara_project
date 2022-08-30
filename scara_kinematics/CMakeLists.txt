cmake_minimum_required(VERSION 3.0.2)
project(scara_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

 add_service_files(
   FILES
   InverseKinematics.srv
   FwdVelocityKinematics.srv
   InvVelocityKinematics.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES scara_kinematics
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


 catkin_install_python(PROGRAMS
   scripts/scara_fwd_kinematics.py
   scripts/scara_inverse_kinematics_client.py
   scripts/scara_inverse_kinematics_server.py
   scripts/scara_velocity_kinematics.py
   scripts/scara_velocity_controller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


