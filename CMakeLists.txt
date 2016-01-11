cmake_minimum_required(VERSION 2.8.3)
project(mtracker)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_srvs tf joy)
#find_package(Boost REQUIRED COMPONENTS xxx)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs tf joy
  DEPENDS system_lib)

include_directories(include ${catkin_INCLUDE_DIRS}) #${Boost_INCLUDE_DIRS}

#add_executable(mtracker src/mtracker.cpp src/rs232.c
#               include/serial.h include/rs232.h)
#target_link_libraries(mtracker ${catkin_LIBRARIES})

add_executable(manual_controller src/manual_controller.cpp include/manual_controller.h)
target_link_libraries(manual_controller ${catkin_LIBRARIES})

add_executable(automatic_controller src/automatic_controller.cpp include/automatic_controller.h)
target_link_libraries(automatic_controller ${catkin_LIBRARIES})

add_executable(reference_generator src/reference_generator.cpp include/reference_generator.h include/trajectories.h)
target_link_libraries(reference_generator ${catkin_LIBRARIES})

add_executable(simulator src/simulator.cpp include/simulator.h)
target_link_libraries(simulator ${catkin_LIBRARIES})

#add_executable(trajectory_recorder src/trajectory_recorder.cpp include/trajectory_recorder.h)
#target_link_libraries(trajectory_recorder ${catkin_LIBRARIES} /usr/local/lib/libyaml-cpp.a)
