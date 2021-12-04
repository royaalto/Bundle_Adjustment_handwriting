cmake_minimum_required(VERSION 2.8)
project(ba_evaluate)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")
find_package(Eigen3 REQUIRED)


#link to g2o
find_package(G2O REQUIRED)
if(G2O_FOUND)
  include_directories(${G2O_INCLUDE_DIR})
  message("G2O lib is found:"${G2O_INCLUDE_DIR})
endif(G2O_FOUND)
set(G2O_LIBS
    g2o_cli
    g2o_ext_freeglut_minimal
    g2o_simulator
    g2o_solver_slam2d_linear
    g2o_types_icp
    g2o_types_slam2d
    g2o_core
    g2o_interface
    g2o_solver_csparse
    g2o_solver_structure_only
    g2o_types_sba
    g2o_types_slam3d
    g2o_csparse_extension
    g2o_opengl_helper
    g2o_solver_dense
    g2o_stuff
    g2o_types_sclam2d
    g2o_parser
    g2o_solver_pcg
    g2o_types_data
    g2o_types_sim3
    cxsparse)
# link to ceres for future
find_package(Ceres Required)
# link to OpenCV for future
find_package(Opencv Required)

include_directories(/usr/local/include/go2/EXTERNAL/csparse)

add_executable(ba_evaluate ba_eigen.cpp ba_g2o.cpp ba_evaluate.cpp)
target_link_libraries(ba_evaluate Eigen3::Eigen ${G2O_LIBS} fmt)
