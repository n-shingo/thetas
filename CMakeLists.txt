cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(camera)

#include_directories( ../../include )

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set the build type
set(CMAKE_BUILD_TYPE Release)
# confirmation messages
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

find_package(PkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv)

add_executable(thetas-handler ./src/thetas-handler.cpp)
target_link_libraries(thetas-handler ssm /usr/local/lib/kaw_lib.a ${OPENCV_LIBRARIES})

add_executable(thetas-viewer ./src/thetas-viewer.cpp)
target_link_libraries(thetas-viewer ssm /usr/local/lib/kaw_lib.a ${OPENCV_LIBRARIES})

