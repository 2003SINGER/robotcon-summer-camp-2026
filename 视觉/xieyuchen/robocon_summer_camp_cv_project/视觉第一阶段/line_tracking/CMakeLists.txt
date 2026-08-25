cmake_minimum_required(VERSION 3.10)
project(line_tracking)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    include_directories(${OpenCV_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OpenCV not found. Please install libopencv-dev.")
endif()

add_executable(line_track main.cpp line_track.cpp)

target_link_libraries(line_track ${OpenCV_LIBS})
