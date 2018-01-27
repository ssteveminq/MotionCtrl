cmake_minimum_required(VERSION 2.8.12)

project(valkyrie)

find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui gui-osg CONFIG)

include_directories(${DART_INCLUDE_DIRS})
include_directories("${PROJECT_SOURCE_DIR}/Controller")
include_directories("${PROJECT_SOURCE_DIR}/Simulator")

add_subdirectory(Controller)
add_subdirectory(Simulator)

add_compile_options(-std=c++11)
