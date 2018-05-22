cmake_minimum_required( VERSION 3.5.1 )

project(plane_sweep VERSION 0.1)

set(CMAKE_CXX_STANDARD 11)

# Compiler flags
set(compiler_options -Wall -Wextra -Wpedantic)

# Find libraries
find_package(OpenCV 3.3.1 REQUIRED)
find_package(sophus REQUIRED)
find_package(GeographicLib REQUIRED)

set(header_files
  include/plane_sweep/attitude.h
  include/plane_sweep/cartesian_position.h
  include/plane_sweep/dataset.h
  include/plane_sweep/feature_utils.h
  include/plane_sweep/geodetic_position.h
  include/plane_sweep/homography_estimator.h
  include/plane_sweep/intrinsics.h
  include/plane_sweep/local_coordinate_system.h
  include/plane_sweep/project.h
  include/plane_sweep/perspective_camera_model.h
  include/plane_sweep/plane_sweep.h
  include/plane_sweep/viewer_3d.h
  )

set(source_files
  src/attitude.cpp
  src/cartesian_position.cpp
  src/dataset.cpp
  src/geodetic_position.cpp
  src/intrinsics.cpp
  src/project.cpp
  src/local_coordinate_system.cpp
  src/perspective_camera_model.cpp
  src/viewer_3d.cpp
  src/homography_estimator.cpp
  src/feature_utils.cpp
  src/plane_sweep.cpp
  )

add_executable(${PROJECT_NAME} examples/main.cpp ${source_files} ${header_files})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${GeographicLib_LIBRARIES})
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${sophus_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE ${compiler_options})
