cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(camera-model VERSION 1.10.4.5 LANGUAGES CXX C)

# check if the correct version of hdf5 available
set(H5CPP_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})
set(HDF5_FIND_DEBUG FALSE)
include(FindHDF5)
if(HDF5_VERSION VERSION_LESS ${H5CPP_VERSION})
	message(FATAL_ERROR "H5CPP examples require of HDF5 v${H5CPP_VERSION} or greater!!!")
else()
	message("-- H5CPP ${PROJECT_VERSION} matches with minimum required HDF5 v${HDF5_VERSION}")
endif()
if(HDF5_VERSION VERSION_GREATER 1.12.0)
	message("H5CPP KITA support is available, see webpage for details...")
endif()

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(h5cpp  REQUIRED)
find_package(VTK REQUIRED COMPONENTS 
  vtkCommonColor
  vtkCommonCore
  vtkFiltersSources
  vtkInteractionStyle
  vtkRenderingOpenGL
  vtkIOImage
  vtkInteractionImage
  vtkGUISupportQt)
message("VTK_QT_VERSION: ${VTK_QT_VERSION}")

find_package(Qt5 COMPONENTS Core REQUIRED QUIET)


include(${VTK_USE_FILE})



list(APPEND LIBS ${HDF5_C_LIBRARIES} ${EIGEN3_LIBRARIES} ${VTK_LIBRARIES})
list(APPEND INCLUDE_DIRS ${CMAKE_SOURCE_DIR} ${HDF5_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

add_executable(main "main.cpp")
qt5_use_modules(main Core Gui Widgets)
target_link_libraries(main ${LIBS} Eigen3::Eigen )
