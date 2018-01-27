file(GLOB srcs "*.cpp" "*.hpp")
add_compile_options(-std=c++11)
add_library(dartController ${srcs})
target_link_libraries(dartController ${DART_LIBRARIES})
