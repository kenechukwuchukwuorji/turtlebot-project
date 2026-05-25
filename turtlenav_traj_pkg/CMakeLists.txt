cmake_minimum_required(VERSION 3.8)
project(turtlenav_traj_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(turtle_nav src/turtle_nav.cpp)
ament_target_dependencies(turtle_nav
  rclcpp
  nav_msgs
  geometry_msgs
)

install(
  DIRECTORY modules
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  turtle_nav
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS script/dubins_path.py
  DESTINATION lib/${PROJECT_NAME}

)


ament_package()
