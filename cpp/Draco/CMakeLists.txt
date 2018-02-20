#cmake_minimum_required(VERSION 2.8.12)

#project(KR5)

#find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui CONFIG)

#include_directories(${DART_INCLUDE_DIRS})
#include_directories ("${PROJECT_SOURCE_DIR}/")
#include_directories ("${PROJECT_SOURCE_DIR}/ControlSet")
#include_directories ("${PROJECT_SOURCE_DIR}/TaskSet")
#include_directories ("${PROJECT_SOURCE_DIR}/StateEstimation")

#add_compile_options(-std=c++11)

#file(GLOB srcs "*.cpp" "*.hpp"
               #"ControlSet/*.cpp" "ControlSet/*.hpp"
               #"TaskSet/*.cpp" "TaskSet/*.hpp"
               #"StateEstimation/*.cpp" "StateEstimation/*.hpp")
#add_executable(${PROJECT_NAME} ${srcs})
#target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})



##### TEMPERARY BUILD
cmake_minimum_required(VERSION 2.8.12)

project(draco)

#find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui gui-osg CONFIG)
#find_package(DART 7.0.0 REQUIRED COMPONENTS io-urdf gui gui-osg CONFIG)
find_package(DART 6.4 REQUIRED COMPONENTS utils-urdf gui gui-osg CONFIG)


include_directories(${DART_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/")
include_directories ("${PROJECT_SOURCE_DIR}/StateEstimation")
include_directories ("${PROJECT_SOURCE_DIR}/WorldNode")

add_compile_options(-std=c++11)

file(GLOB srcs "*.cpp" "*.hpp"
               "WorldNode/*.cpp" "WorldNode/*.hpp"
               "StateEstimation/*.cpp" "StateEstimation/*.hpp")

add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
