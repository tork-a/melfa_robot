cmake_minimum_required(VERSION 2.8.3)
project(melfa_description)

find_package(catkin REQUIRED COMPONENTS
  rviz
  xacro
)

catkin_package()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(DIRECTORY launch mesh urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(status "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
endif()
