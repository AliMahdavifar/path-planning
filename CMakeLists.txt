#Project name
project(PathPlanning)
cmake_policy(VERSION 3.5)
#Version
cmake_minimum_required (VERSION 3.5)
add_definitions(-std=c++11 -g  -Wall)

#Enable warnings
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

execute_process(COMMAND git submodule update --init -- external/yaml-cpp
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

execute_process(COMMAND git submodule update --init -- external/googletest
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "Enable testing" FORCE)
add_subdirectory(external/yaml-cpp)

#src files
# file(GLOB sources "./*.cpp")
file(GLOB sources "./src/*.cpp")
ADD_LIBRARY(LibsModule
    ${sources}
)

target_link_libraries(LibsModule yaml-cpp)

#header files
include_directories(include)

#Destination file
add_executable(pathplanner ${sources})
target_link_libraries(pathplanner LibsModule)
# target_link_libraries(yaml-cpp )

add_subdirectory(external/googletest)

enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
file(GLOB TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/test/*.cpp)
add_executable(runUnitTests ${TEST_SRC_FILES})
target_link_libraries(runUnitTests gtest gtest_main)