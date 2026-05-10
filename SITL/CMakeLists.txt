cmake_minimum_required(VERSION 3.16)
project(SAR_SITL CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -O2)

# ── Include paths ─────────────────────────────────────────────────
# include/          — nlohmann/json and other third-party headers
# .                 — packets.h, config.h, transport.h, json_parser.h
#                     + all swarm headers (KalmanFilter.h etc.)
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

# ── Original drone_controller target (unchanged) ──────────────────
add_executable(drone_controller
    src/main.cpp
    controllers/DroneController.cpp
)

# ── Swarm controller target ───────────────────────────────────────
# All swarm headers are header-only — only one .cpp needed
add_executable(swarm_controller
    src/swarm_main.cpp
)

# ── Link threads ──────────────────────────────────────────────────
find_package(Threads REQUIRED)
target_link_libraries(drone_controller  Threads::Threads)
target_link_libraries(swarm_controller Threads::Threads)

if(WIN32)
    target_link_libraries(drone_controller  ws2_32)
    target_link_libraries(swarm_controller ws2_32)
endif()

# ── Zone file: copy next to binary if present ────────────────────
set(ZONE_SRC "${CMAKE_SOURCE_DIR}/anchorage_zone.json")
if(EXISTS "${ZONE_SRC}")
    configure_file("${ZONE_SRC}" "${CMAKE_BINARY_DIR}/anchorage_zone.json" COPYONLY)
else()
    message(WARNING "anchorage_zone.json not found — copy it to ${CMAKE_BINARY_DIR}/ before running")
endif()

message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "Source dir : ${CMAKE_SOURCE_DIR}")
message(STATUS "Binary dir : ${CMAKE_BINARY_DIR}")