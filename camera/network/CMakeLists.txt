cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

add_library(network output.cpp net_output.cpp net_input.cpp)

install(TARGETS network LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

