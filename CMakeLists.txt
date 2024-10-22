cmake_minimum_required(VERSION 3.5.0)
project(asc2pcd LANGUAGES C CXX)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -pthread")
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-O0 -Wall -Wextra -g -pthread")

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 0)
set(${PROJECT_NAME}_PATCH_VERSION 1)
set(${PROJECT_NAME}_VERSION
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})
  

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(glog REQUIRED)

add_executable(asc2pcd src/main.cpp)

target_link_libraries(
    asc2pcd
    ${PCL_LIBRARIES}
    glog::glog
)