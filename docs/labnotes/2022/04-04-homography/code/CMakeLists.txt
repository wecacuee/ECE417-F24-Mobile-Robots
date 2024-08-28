cmake_minimum_required (VERSION 3.0)
project (homography)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (OpenCV 4.2 REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
add_executable (homography homography.cpp)
target_link_libraries (homography
  Eigen3::Eigen
  ${OpenCV_LIBRARIES})
