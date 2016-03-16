cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
include_directories(/usr/include/flycapture)

add_executable( fireflytest fireflyview.cpp )
target_link_libraries( fireflytest ${OpenCV_LIBS} flycapture )
