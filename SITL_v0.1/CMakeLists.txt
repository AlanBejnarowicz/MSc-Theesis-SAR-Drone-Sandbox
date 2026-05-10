cmake_minimum_required(VERSION 3.16)
project(DroneController)

set(CMAKE_CXX_STANDARD 17)

if(WIN32)
    set(PLATFORM_LIBS ws2_32)
else()
    set(PLATFORM_LIBS pthread)
endif()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers      
)

add_executable(drone_controller
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers/DroneController.cpp 
)

target_link_libraries(drone_controller ${PLATFORM_LIBS})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()