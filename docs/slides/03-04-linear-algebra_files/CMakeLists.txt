cmake_minimum_required (VERSION 3.0)
project (findeig)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (findeig findeig.cpp)
target_link_libraries (findeig Eigen3::Eigen)
