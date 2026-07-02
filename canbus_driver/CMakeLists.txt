cmake_minimum_required(VERSION 3.12) # 1. Higher version fixes the CMake deprecation warning
project(canbus_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # 2. Bumped to C++17 (standard for modern ROS 2 layouts)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find libraries
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # 3. CRITICAL: Required for ament_python_install_package
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Install Python modules
# NOTE: Only keep this line if you have a folder named 'smartec_motor_driver' 
# inside your package root containing an __init__.py file. 
# If 'canbus_driver.py' is just a standalone script, you can safely comment this out.

# Install Python executables
install(PROGRAMS
  scripts/canbus_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/steering_can.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/roboteq_velocity_sender.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
