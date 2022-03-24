cmake_minimum_required (VERSION 3.0)
project (hw5)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (hw5 hw5.cpp)
target_link_libraries (hw5 Eigen3::Eigen)
