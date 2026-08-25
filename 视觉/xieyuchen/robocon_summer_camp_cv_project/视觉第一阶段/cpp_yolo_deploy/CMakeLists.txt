cmake_minimum_required(VERSION 3.10)
project(YoloInference)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(include)

# 标定程序（只需要运行一次）
add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS})

# 主程序（接收Python坐标并计算）
add_executable(yolo_inference src/main.cpp src/Uart.cpp)
target_link_libraries(yolo_inference ${OpenCV_LIBS} serial)
