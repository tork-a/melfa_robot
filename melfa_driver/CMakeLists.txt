cmake_minimum_required(VERSION 2.8.3)
project(melfa_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
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

target_link_libraries(melfa_driver_node ${catkin_LIBRARIES})

install(TARGETS melfa_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

foreach(dir launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
