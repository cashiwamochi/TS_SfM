cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ST_SfM)

set(CMAKE_CXX_COMPILER g++)

if(UNIX)
  add_definitions(-DUNIX)
  message(STATUS "Compiling on Unix")
endif(UNIX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/thirdparty/g2o/cmake_modules)

set(OpenCV_DIR ~/.local/OpenCV410/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
IF (OpenCV_FOUND)
  message([OpenCV_VER]: ${OpenCV_VERSION})
  message([OpenCV_INCLUDE_DIRS]: ${OpenCV_INCLUDE_DIRS})
  message([OpenCV_LIBRARIES]: ${OpenCV_LIBRARIES})
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

find_package(Cholmod REQUIRED)
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)

set(G2O_ROOT ${CMAKE_SOURCE_DIR}/thirdparty/g2o)
set(IMGUI_ROOT ${CMAKE_SOURCE_DIR}/thirdparty/imgui)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR}
  ${G2O_ROOT}
  ${G2O_ROOT}/build # for g2o/config.h
  ${IMGUI_ROOT}
  ${IMGUI_ROOT}/examples
  ${IMGUI_ROOT}/examples/libs/gl3w
  ${IMGUI_ROOT}/examples/libs/glfw
  /usr/include/eigen3
)

link_directories(
  ${G2O_ROOT}/lib
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -Wall -O2 -DIMGUI_IMPL_OPENGL_LOADER_GL3W")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++14 -O2 -DIMGUI_IMPL_OPENGL_LOADER_GL3W") 


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(
  ImGui STATIC
  ${IMGUI_ROOT}/imconfig.h
  ${IMGUI_ROOT}/imgui_demo.cpp
  ${IMGUI_ROOT}/imgui_draw.cpp
  ${IMGUI_ROOT}/examples/imgui_impl_glfw.cpp
  ${IMGUI_ROOT}/examples/imgui_impl_glfw.h
  ${IMGUI_ROOT}/examples/imgui_impl_opengl3.cpp
  ${IMGUI_ROOT}/examples/imgui_impl_opengl3.h
  ${IMGUI_ROOT}/examples/libs/gl3w/GL/gl3w.c 
  ${IMGUI_ROOT}/examples/libs/glfw/include/GLFW/glfw3.h
  ${IMGUI_ROOT}/imgui_internal.h
  ${IMGUI_ROOT}/imgui_widgets.cpp
  ${IMGUI_ROOT}/imgui.cpp
  ${IMGUI_ROOT}/imgui.h
  ${IMGUI_ROOT}/imstb_rectpack.h
  ${IMGUI_ROOT}/imstb_textedit.h
  ${IMGUI_ROOT}/imstb_truetype.h
)

target_link_libraries(
  ImGui
  glfw
  GL
  ${CMAKE_DL_LIBS}
)

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
  ImGui
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
