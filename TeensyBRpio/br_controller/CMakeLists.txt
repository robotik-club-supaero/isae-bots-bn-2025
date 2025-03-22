set (CMAKE_CXX_STANDARD 20)

cmake_minimum_required(VERSION 3.5)
project (br_simu)

add_compile_definitions(_SIMULATION)
add_compile_definitions(_BR_DEBUG)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(br_messages REQUIRED)
# find_package(...) # Add dependencies here

include_directories(include)

file(GLOB_RECURSE base_SRCS
   "${PROJECT_SOURCE_DIR}/src/controller/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/feedback/UnicycleStateSimulator.cpp" 
    "${PROJECT_SOURCE_DIR}/src/feedback/TwoWheelSimulator.cpp" 
    "${PROJECT_SOURCE_DIR}/src/geometry/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/manager/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/math/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/motors/MotorStub.cpp" 
    "${PROJECT_SOURCE_DIR}/src/rotations/*.cpp" 
    "${PROJECT_SOURCE_DIR}/src/trajectories/*.cpp"
)
set(ros_SRCS ${PROJECT_SOURCE_DIR}/src/ros/ROS.cpp ${PROJECT_SOURCE_DIR}/src/ros/Dispatcher.cpp)
set(test_SRCS ${PROJECT_SOURCE_DIR}/test/test_logging.cpp)

add_executable(simulation_br src/main_simulation.cpp ${base_SRCS} ${ros_SRCS})
target_link_libraries(simulation_br m)
ament_target_dependencies(simulation_br rclcpp std_msgs br_messages)

install(TARGETS
  simulation_br
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_curve test/test_curve.cpp ${base_SRCS} ${test_SRCS})
  target_link_libraries(test_curve m sfml-graphics sfml-window sfml-system)
  target_include_directories(test_curve PRIVATE test)

  add_executable(test_speed_control test/test_speed_control.cpp ${base_SRCS} ${test_SRCS})
  target_link_libraries(test_speed_control m sfml-graphics sfml-window sfml-system)
  target_include_directories(test_speed_control PRIVATE test)

  install(TARGETS
    test_curve test_speed_control
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()