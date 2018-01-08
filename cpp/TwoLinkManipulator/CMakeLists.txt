cmake_minimum_required(VERSION 2.8.12)

project(TwoLinkage)

find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui CONFIG)

include_directories(${DART_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/")
include_directories ("${PROJECT_SOURCE_DIR}/ControlSet")
include_directories ("${PROJECT_SOURCE_DIR}/TaskSet")
include_directories ("${PROJECT_SOURCE_DIR}/StateEstimation")

add_compile_options(-std=c++11)

file(GLOB srcs "*.cpp" "*.hpp"
               "ControlSet/*.cpp" "ControlSet/*.hpp"
               "TaskSet/*.cpp" "TaskSet/*.hpp"
               "StateEstimation/*.cpp" "StateEstimation/*.hpp")
add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
