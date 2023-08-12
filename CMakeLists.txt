cmake_minimum_required(VERSION 2.8.12)

project(boostMAV)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)
find_package(MAVLink)

include_directories(include)


add_executable(boostMAV src/main.cpp src/mavlink_interface.cpp)
target_include_directories(boostMAV
                BEFORE
                                PUBLIC ${MAVLINK_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

target_link_libraries(boostMAV ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_TIMER_LIBRARY_RELEASE})
