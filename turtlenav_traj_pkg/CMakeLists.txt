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
<<<<<<< HEAD
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
=======

>>>>>>> a32b03120e5bec804cef30018d7af3872a61f63d

add_executable(turtle_nav src/turtle_nav.cpp)
ament_target_dependencies(turtle_nav
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)

<<<<<<< HEAD
add_executable(a_star_lite src/a_star_lite.cpp)
ament_target_dependencies(a_star_lite
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)
=======
install(
  DIRECTORY modules
  DESTINATION lib/${PROJECT_NAME}
)

>>>>>>> a32b03120e5bec804cef30018d7af3872a61f63d

install(TARGETS
  turtle_nav
  a_star_lite
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS script/dubins_path.py
  DESTINATION lib/${PROJECT_NAME}

)


ament_package()
