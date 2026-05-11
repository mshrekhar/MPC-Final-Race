cmake_minimum_required(VERSION 3.8)
project(frenet_mpc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# acados paths. Adjust only if your acados install is somewhere else.
set(ACADOS_SOURCE_DIR "$ENV{HOME}/acados")
set(ACADOS_INCLUDE_DIR "${ACADOS_SOURCE_DIR}/include")
set(ACADOS_LIB_DIR "${ACADOS_SOURCE_DIR}/lib")
set(FRENET_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_frenet")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Build frenet_mpc_node only after scripts/generate_frenet_solver.py has generated c_generated_code_frenet/.
if(EXISTS "${FRENET_GENERATED_DIR}/acados_solver_f1tenth_frenet.h")
  message(STATUS "Frenet acados generated code found — building frenet_mpc_node")

  file(GLOB_RECURSE FRENET_GENERATED_SOURCES "${FRENET_GENERATED_DIR}/*.c")

  add_library(f1tenth_frenet_solver STATIC ${FRENET_GENERATED_SOURCES})
  target_include_directories(f1tenth_frenet_solver PUBLIC
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
    ${FRENET_GENERATED_DIR}
  )
  target_link_libraries(f1tenth_frenet_solver
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    m
  )

  add_executable(frenet_mpc_node src/frenet_mpc_node.cpp)

  target_include_directories(frenet_mpc_node PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
    ${FRENET_GENERATED_DIR}
  )

  ament_target_dependencies(frenet_mpc_node
    rclcpp
    geometry_msgs
    nav_msgs
    ackermann_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
  )

  target_link_libraries(frenet_mpc_node
    f1tenth_frenet_solver
    ${ACADOS_LIB_DIR}/libacados.so
    ${ACADOS_LIB_DIR}/libblasfeo.so
    ${ACADOS_LIB_DIR}/libhpipm.so
    m
  )

  install(TARGETS frenet_mpc_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "Frenet acados generated code NOT found — skipping frenet_mpc_node")
  message(STATUS "Run: cd ${CMAKE_CURRENT_SOURCE_DIR} && python3 scripts/generate_frenet_solver.py")
endif()

install(PROGRAMS
  scripts/generate_frenet_solver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
