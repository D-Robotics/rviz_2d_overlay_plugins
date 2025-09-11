cmake_minimum_required(VERSION 3.8)
project(rviz_2d_overlay_bridge)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(${PROJECT_NAME} src/string_to_overlay_text.cpp)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        diagnostic_msgs
        rviz_2d_overlay_msgs
)

install(
        TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
