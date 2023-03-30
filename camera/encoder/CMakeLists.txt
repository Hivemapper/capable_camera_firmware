cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

include_directories(${CMAKE_SOURCE_DIR} )
link_directories(${CMAKE_SOURCE_DIR} )

add_library(encoders encoder.cpp null_encoder.cpp h264_encoder.cpp mjpeg_encoder.cpp)
target_link_libraries(encoders jpeg yuv exif fmt)

install(TARGETS encoders LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

