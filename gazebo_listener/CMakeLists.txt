cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo_listener)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

FILE(GLOB SOURCE_FILES ./*.cc)

add_executable(listener ${SOURCE_FILES})
target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread)
