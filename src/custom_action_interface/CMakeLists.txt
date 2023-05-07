cmake_minimum_required(VERSION 3.5)
project(custom_action_interface)

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

##============================================================
##  Find and load build settings from external packages
##============================================================
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED) ## added
find_package(rosidl_default_generators REQUIRED)## added
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
##============================================================



##============================================================
##  Declare ROS messages, Fibonacci action
##============================================================
set(action_files
 "action/Fibonacci.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)
##============================================================



##============================================================
## Macro for ament package
##============================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()
##============================================================
