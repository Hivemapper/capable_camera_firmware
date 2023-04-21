cmake_minimum_required(VERSION 3.6)

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS filesystem)

add_library(network output.cpp net_input.cpp file_output.cpp file_manager.cpp)

target_link_libraries(network ${Boost_LIBRARIES} fmt)

install(TARGETS network LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

