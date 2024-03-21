cmake_minimum_required(VERSION 3.5)
project(uwb_locate)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_interfaces REQUIRED)
find_package(gz-transport12 QUIET REQUIRED OPTIONAL_COMPONENTS log)

find_package( PkgConfig REQUIRED)
pkg_check_modules( gsl REQUIRED gsl )

add_executable(uwb_location
  src/uwb_location_main.cpp 
  src/uwb_location.cpp
  src/tinyxml2/tinyxml2.cpp
  src/ransac_locator/ransac_locator.cpp
  )
  
target_include_directories(uwb_location PUBLIC include)

target_link_libraries(uwb_location ${gsl_LIBRARIES} )

ament_target_dependencies(uwb_location rclcpp nav_msgs ament_index_cpp uwb_interfaces)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  uwb_location
  DESTINATION lib/${PROJECT_NAME})

add_executable(gz_forwarder
src/gz_forwarder.cpp 
)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})
target_link_libraries(gz_forwarder gz-transport${GZ_TRANSPORT_VER}::core)

ament_target_dependencies(gz_forwarder rclcpp ament_index_cpp geometry_msgs)

install(TARGETS
gz_forwarder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
    scripts/locate.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
