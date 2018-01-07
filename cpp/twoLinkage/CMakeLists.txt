cmake_minimum_required(VERSION 2.8.12)

project(twoLinkage)

find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui CONFIG)

add_subdirectory(Controller)

include_directories(${DART_INCLUDE_DIRS})
