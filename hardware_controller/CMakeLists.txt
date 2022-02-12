cmake_minimum_required(VERSION 3.8)
project(hardware_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(hardware_node src/hardware_node.cpp)
target_include_directories(hardware_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hardware_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(hardware_node Threads rclcpp can_msgs sensor_msgs)

#link CTRE libs
set(CTRE_EXEC hardware_node)
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
	  target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCore.a)
    target_link_libraries(${CTRE_EXEC} -lstdc++fs)
elseif (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_Phoenix.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCCI.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/jetsontx/libCTRE_PhoenixCore.a)
else()
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixDiagnostics.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatform_socketcan.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
    target_link_libraries(${CTRE_EXEC} ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCore.a)
endif()


install(TARGETS hardware_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()
