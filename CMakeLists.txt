cmake_minimum_required(VERSION 2.8.3)
project(mtracker)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf joy message_generation)

#find_package(Boost REQUIRED COMPONENTS system)

#add_message_files(FILES Controls.msg)
add_service_files(FILES Trigger.srv)
#add_action_files(FILES Action1.action)

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include/mtracker
  CATKIN_DEPENDS roscpp geometry_msgs tf joy message_runtime
  DEPENDS system_lib
)

include_directories(
  #${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include/mtracker
)

# The bridge between high level and low level controllers
add_executable(mtracker 
  src/mtracker.cpp  src/rs232.c
  include/mtracker/Robot.hpp
  include/mtracker/Serial.hpp
  include/mtracker/rs232.h
)
add_dependencies(mtracker mtracker_generate_messages_cpp)
target_link_libraries(mtracker ${catkin_LIBRARIES})

# An application of manual joystick controller
add_executable(manual_controller
  src/manual_controller.cpp
)
add_dependencies(manual_controller mtracker_generate_messages_cpp)
target_link_libraries(manual_controller ${catkin_LIBRARIES})

# An application of automatic controller
add_executable(automatic_controller
  src/automatic_controller.cpp
)
add_dependencies(automatic_controller mtracker_generate_messages_cpp)
target_link_libraries(automatic_controller ${catkin_LIBRARIES})

# The reference trajectory generator for the automatic_controller
add_executable(reference_generator
  src/reference_generator.cpp
)
add_dependencies(reference_generator mtracker_generate_messages_cpp)
target_link_libraries(reference_generator ${catkin_LIBRARIES})

# A simulator of MTracker
add_executable(simulator
  src/simulator.cpp
)
add_dependencies(simulator mtracker_generate_messages_cpp)
target_link_libraries(simulator ${catkin_LIBRARIES})

# A Trajectory recorder
#add_executable(trajectory_recorder
#  src/trajectory_recorder.cpp
#)
#add_dependencies(trajectory_recorder mtracker_generate_messages_cpp)
#target_link_libraries(trajectory_recorder ${catkin_LIBRARIES})

