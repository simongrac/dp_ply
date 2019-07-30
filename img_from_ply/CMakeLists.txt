cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(img_from_ply)
SET(OpenCV_DIR /home/simon/installation/OpenCV-3.4.4/share/OpenCV/)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED)


message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable (img_from_ply main.cpp)
target_link_libraries (img_from_ply ${PCL_LIBRARIES})
target_link_libraries(img_from_ply  ${OpenCV_LIBS})