cmake_minimum_required(VERSION 3.5)
project(ttlidar_node_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(TTLIDAR_SDK_PATH "./sdk/")

include_directories(
  ${TTLIDAR_SDK_PATH}/include
  ${TTLIDAR_SDK_PATH}/src
  ${TTLIDAR_SDK_PATH}/src/arch
  ${TTLIDAR_SDK_PATH}/src/arch/linux

)

file(GLOB TTLIDAR_SDK_SRC 
  "${TTLIDAR_SDK_PATH}/src/*.cpp"
  "${TTLIDAR_SDK_PATH}/src/arch/*.cpp" 
  "${TTLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
)


add_executable(ttlidar_node src/ttlidar_node.cpp ${TTLIDAR_SDK_SRC})
ament_target_dependencies(ttlidar_node  
    rclcpp
    sensor_msgs
    std_srvs
)


add_executable(subscribe_demo_node src/subscribe_demo_node.cpp)
ament_target_dependencies(subscribe_demo_node
    rclcpp
    sensor_msgs
    std_srvs
)


install(TARGETS
    ttlidar_node subscribe_demo_node
    RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch  rviz 
    DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_include_directories(include)
ament_package()




