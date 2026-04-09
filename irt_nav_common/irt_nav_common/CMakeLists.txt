cmake_minimum_required(VERSION 3.8)
project(irt_nav_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options( -Wall -Wextra -Wpedantic -std=c++17)
endif()

#set(CMAKE_CXX_FLAGS_RELEASE "-g -pthread")

# path for custom find scripts, use same of libRSF
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irt_nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(GTSAMCMakeTools)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3.5 REQUIRED NO_MODULE)

include_directories(
        include
        include/${PROJECT_NAME}
)


set(lib_source_files
        src/navigation_tools.cpp)

add_library(${PROJECT_NAME} SHARED
        ${lib_source_files})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        )

target_link_libraries(${PROJECT_NAME}
        gtsam
        gtsam_unstable
        Eigen3::Eigen
        pthread
)

ament_target_dependencies(
        ${PROJECT_NAME}
        "rclcpp"
        "std_msgs"
        "sensor_msgs"
        "irt_nav_msgs"
        "tf2_geometry_msgs"
        "geometry_msgs"
)

install(
        DIRECTORY "include/"
        DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin}
        )
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
