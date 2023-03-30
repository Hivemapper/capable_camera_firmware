cmake_minimum_required(VERSION 3.6)

project(dashcam)

set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)

add_subdirectory(common)

add_subdirectory(led)
add_subdirectory(gnss)
add_subdirectory(imu)