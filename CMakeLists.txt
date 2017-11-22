cmake_minimum_required( VERSION 2.8 )
project( kalman_tracking_mouse )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

#指定opencv安装位置
set(CMAKE_PREFIX_PATH "/usr/local/opencv3.2/share/OpenCV")
FIND_PACKAGE(OpenCV 3.2 REQUIRED)
message("hahainclude_dir = " ${OpenCV_INCLUDE_DIRS})

add_executable( opencvLib_tracking MouseTrack.cpp )
target_link_libraries(opencvLib_tracking ${OpenCV_LIBS})



add_executable( eigen_kalman_tracking mouse_tracking_eigen.cpp )
target_link_libraries(eigen_kalman_tracking ${OpenCV_LIBS})
