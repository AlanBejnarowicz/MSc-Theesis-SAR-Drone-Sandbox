cmake_minimum_required(VERSION 3.16)
project(kalman_nav CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -O2)

# ── Include paths ─────────────────────────────────────────────────
# .        = KalmanNav.h + kalman_main.cpp
# SITL_DIR = packets.h, transport.h, json_parser.h, config.h
#            (point this at your existing SITL source root)
set(SITL_DIR "${CMAKE_SOURCE_DIR}/../"
    CACHE PATH "Path to SITL directory containing packets.h etc.")

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${SITL_DIR}
    ${SITL_DIR}/include          # nlohmann/json
)

add_executable(kalman_nav kalman_main.cpp)

find_package(Threads REQUIRED)
target_link_libraries(kalman_nav Threads::Threads)

if(WIN32)
    target_link_libraries(kalman_nav ws2_32)
endif()

message(STATUS "Build : ${CMAKE_BUILD_TYPE}")
message(STATUS "SITL  : ${SITL_DIR}")
