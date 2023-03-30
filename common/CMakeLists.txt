cmake_minimum_required(VERSION 3.6)

project(common)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(nlohmann_json REQUIRED)

set(CMAKE_CXX_STANDARD 17)

file(GLOB PROJECT_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SRC}
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${nlohmann_json_LIBRARIES}
    pthread
)