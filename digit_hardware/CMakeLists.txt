cmake_minimum_required(VERSION 3.8)
project(digit_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)

# Set paths
set(LIBRARY_PATH "${CMAKE_SOURCE_DIR}/thirdparty/api_AR/lib")
set(INCLUDE_PATH "${CMAKE_SOURCE_DIR}/thirdparty/api_AR/include")
set(SRC_PATH "${CMAKE_SOURCE_DIR}/thirdparty/api_AR/src")

# Include necessary directories
include_directories(
    include
)

# Compile the low-level API into a static library
add_library(AR_api STATIC ${SRC_PATH}/lowlevelapi.c)
target_include_directories(AR_api PUBLIC ${INCLUDE_PATH})
target_link_libraries(AR_api PUBLIC -lpthread)

# Define executable
add_executable(DigitCommunicationNode src/digit_comm.cpp)

# Link the static library properly
target_link_libraries(DigitCommunicationNode AR_api ${LIBRARY_PATH}/libartl.a -lpthread)

# Ensure rclcpp dependency is included
ament_target_dependencies(DigitCommunicationNode rclcpp std_srvs)

# Install the executable
install (
    TARGETS DigitCommunicationNode
    DESTINATION lib/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/digit_json.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
