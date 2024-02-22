cmake_minimum_required(VERSION 3.5)
project(csm)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

if(WIN32)
  add_definitions(-DWINDOWS)
  if(MSVC)
    # SSE2 optimizations
    add_definitions("/arch:SSE2")
  endif()
endif()


include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

set(CSM_SRC
  src/csm/laser_data.cpp
  src/csm/math_utils.cpp
  src/csm/math_utils_gsl.cpp
  src/csm/utils.cpp
  src/csm/logging.cpp
  src/csm/sm_options.cpp
  src/csm/orientation.cpp
  src/csm/clustering.cpp)

set(EGSL_SRC
  src/egsl/egsl.cpp
  src/egsl/egsl_conversions.cpp
  src/egsl/egsl_misc.cpp
  src/egsl/egsl_ops.cpp)

set(GPC_SRC
  src/gpc/gpc.cpp
  src/gpc/gpc_utils.cpp)

set(ICP_SRC
  src/icp/icp.cpp
  src/icp/icp_corr_dumb.cpp
  src/icp/icp_corr_tricks.cpp
  src/icp/icp_covariance.cpp
  src/icp/icp_debug.cpp
  src/icp/icp_loop.cpp
  src/icp/icp_outliers.cpp)

set(OPTIONS_SRC
  src/options/options.cpp
  src/options/options_interface.cpp)

add_library(${PROJECT_NAME} SHARED
  ${CSM_SRC}
  ${EGSL_SRC}
  ${GPC_SRC}
  ${ICP_SRC}
  ${OPTIONS_SRC})



install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include
  DESTINATION include/../
)

install(DIRECTORY resource
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/gsl_eigen/
  DESTINATION include/../gsl_eigen
)


ament_export_dependencies(rclcpp)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()


