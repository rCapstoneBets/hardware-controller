cmake_minimum_required(VERSION 3.8)
project(hardware_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(hardware_node src/hardware_node.cpp)
target_include_directories(hardware_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hardware_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(hardware_node Threads::Threads)

ament_target_dependencies(hardware_node rclcpp can_msgs sensor_msgs diagnostic_msgs diagnostic_updater)


if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/raspberry)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/jetsontx)
else()
    set(PHOENIX_LIBS_DIR ${CMAKE_SOURCE_DIR}/lib/x86-64)
endif()

target_link_libraries(hardware_node ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so)
target_link_libraries(hardware_node ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so)

install(TARGETS hardware_node
  DESTINATION lib/${PROJECT_NAME})

install(
  FILES ${PHOENIX_LIBS_DIR}/libCTRE_Phoenix.so ${PHOENIX_LIBS_DIR}/libCTRE_PhoenixCCI.so
  DESTINATION lib
)

#install shared dirs
install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}/)


ament_package()
