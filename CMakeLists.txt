cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ST_SfM)

set(CMAKE_CXX_COMPILER g++)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14 -O2")

list(APPEND CMAKE_MODULE_PATH ~/source/g2o/cmake_modules)

set(OpenCV_DIR ~/.local/OpenCV410/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
IF (OpenCV_FOUND)
  message([OpenCV_VER]: ${OpenCV_VERSION})
  message([OpenCV_INCLUDE_DIRS]: ${OpenCV_INCLUDE_DIRS})
  message([OpenCV_LIBRARIES]: ${OpenCV_LIBRARIES})
ENDIF()

# set(Open3D_DIR ~/.local/Open3D/lib/cmake/Open3D/)
set(Open3D_DIR ~/.local/Open3D_1/lib/cmake/Open3D/)
find_package(Open3D REQUIRED)
IF (Open3D_FOUND)
  list(APPEND Open3D_LIBRARIES dl)
  message([Open3D_VER]: ${Open3D_VERSION})
  message([Open3D_INCLUDE_DIRS]: ${Open3D_INCLUDE_DIRS})
  message([Open3D_LIBRARY_DIRS]: ${Open3D_LIBRARY_DIRS})
  message([Open3D_LIBRARIES]: ${Open3D_LIBRARIES})
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Open3D_CXX_FLAGS}")

find_package(Cholmod REQUIRED)
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)

set(G2O_ROOT ~/.local/g2o)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CHOLMOD_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
  ${G2O_ROOT}/include
  /usr/inluce/eigen3
)

link_directories(
  ${G2O_ROOT}/lib
  ${Open3D_LIBRARY_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(
  ${PROJECT_NAME} STATIC
  src/System.cc
  src/Utils.cc
  src/Reconstructor.cc
  src/Tracker.cc
  src/Mapper.cc
  src/Frame.cc
  src/KeyFrame.cc
  src/MapPoint.cc
  src/Map.cc
  src/KPExtractor.cc
  src/Matcher.cc
  src/Solver.cc
  src/Optimizer.cc
  src/Viewer.cc
  src/LoopClosure.cc
  src/ConfigLoader.cc
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Open3D_LIBRARIES}
  libg2o_types_data.so
  libg2o_core.so
  libg2o_solver_cholmod.so
  libg2o_types_sba.so
  libg2o_solver_dense.so
  libg2o_solver_eigen.so
  libg2o_types_sim3.so
  libg2o_solver_structure_only.so
  libg2o_types_slam3d_addons.so
  libg2o_stuff.so
  libg2o_types_slam3d.so
  ${CHOLMOD_LIBRARIES}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build/bin)

add_executable(
  ts_sfm_example
  main.cc
)

target_link_libraries(
  ts_sfm_example
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)
