PROJECT(xmate_test CXX C)

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

SET(CMAKE_BUILD_TYPE "Debug")

message(STATUS "PRO_PATH = " ${PROJECT_SOURCE_DIR})

add_subdirectory(examples)
add_subdirectory(src)

