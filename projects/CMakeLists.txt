cmake_minimum_required(VERSION 3.8)
project(projects)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

set(NODES
  victims_path_planner
)

add_library(${PROJECT_NAME}_lib SHARED
  src/simplex.cpp
  src/ilp_solver.cpp
  src/math_utils.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)

# build nodes
foreach(NODE ${NODES})
  add_executable(${NODE} src/${NODE}.cpp)

  target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
  target_compile_features(${NODE} PUBLIC c_std_99 cxx_std_17)
  
  target_link_libraries(${NODE} ${PROJECT_NAME}_lib)

  ament_target_dependencies(
    ${NODE}
    "geometry_msgs"
    "rclcpp"
    "obstacles_msgs"
    "rclcpp_lifecycle"
  )

  install(TARGETS ${NODE}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
