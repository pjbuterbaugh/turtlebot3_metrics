cmake_minimum_required(VERSION 3.8)
project(turtlebot3_metrics)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(reporter src/get_metrics.cpp)
ament_target_dependencies(reporter rclcpp std_msgs sensor_msgs)

install(TARGETS
	reporter
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
