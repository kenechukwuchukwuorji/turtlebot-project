cmake_minimum_required(VERSION 3.8)
project(cap_detector_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(task4_caps src/task4_caps.cpp)
ament_target_dependencies(task4_caps
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  cv_bridge
)
target_include_directories(task4_caps PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(task4_caps ${OpenCV_LIBS})

install(TARGETS
  task4_caps
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
