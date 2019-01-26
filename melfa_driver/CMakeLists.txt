cmake_minimum_required(VERSION 2.8.3)
project(melfa_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diagnostic_updater
  hardware_interface
  roscpp)

find_package(Boost REQUIRED COMPONENTS system thread)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(melfa_driver_node
  src/melfa_driver_node.cpp src/melfa_hardware_interface.cpp)

add_executable(melfa_loopback_node src/melfa_loopback_node.cpp)

target_link_libraries(melfa_driver_node ${catkin_LIBRARIES})

target_link_libraries(melfa_loopback_node ${catkin_LIBRARIES})

install(TARGETS melfa_driver_node melfa_loopback_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int")
  # roslint_cpp(src/melfa_hardware_interface.cpp src/melfa_driver_node.cpp src/melfa_loopback_node.cpp)
  set(ROSLINT_PYTHON_OPTS "--max-line-length=180")
  # roslint_python(test/test_joint_trajectory_controller.py)
  # add_dependencies(tests roslint)

  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()

  add_rostest(test/joint_trajectory_controller.test)
endif()
