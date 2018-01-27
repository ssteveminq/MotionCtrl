file(GLOB srcs "*.cpp" "*.hpp")
add_compile_options(-std=c++11)
add_executable(${PROJECT_NAME} ${srcs})
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES}
                                      dartController)
