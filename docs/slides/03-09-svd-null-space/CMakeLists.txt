cmake_minimum_required (VERSION 3.0)
project (findeig)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (Python COMPONENTS Interpreter Development NumPy)


include_directories("include")
add_executable (fitline fitline.cpp)
target_link_libraries (fitline
  Eigen3::Eigen
  Python::Python
  Python::Module
  Python::NumPy)

add_executable (fitlinebyeig fitlinebyeig.cpp)
target_link_libraries (fitlinebyeig
  Eigen3::Eigen
  Python::Python
  Python::Module
  Python::NumPy)
