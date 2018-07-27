cmake_minimum_required (VERSION 2.8)
project (edge_align_standalone)

# Needs CERES, Eigen, OpenCV3
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV 3 REQUIRED )


include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

set( CMAKE_CXX_FLAGS "-fpermissive -std=c++11 -O3" )



add_executable( helloDemo hello.cpp )
add_executable( edge_align standalone_edge_align.cpp utils.cpp PoseManipUtils.cpp)

target_link_libraries( helloDemo )
target_link_libraries( edge_align  ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} )
