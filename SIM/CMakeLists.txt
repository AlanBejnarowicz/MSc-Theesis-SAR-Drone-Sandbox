cmake_minimum_required(VERSION 3.11)
project(MarineSIM VERSION 1.0.0)

# Enable C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for better IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Better build types
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Dependencies
set(RAYLIB_VERSION 5.5)
find_package(raylib ${RAYLIB_VERSION} QUIET)
if(NOT raylib_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG ${RAYLIB_VERSION}
    )
    FetchContent_MakeAvailable(raylib)
endif()

# Add all source files in src directory
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE HEADERS "src/*.h" "src/*.hpp")

# Our Project
add_executable(${PROJECT_NAME} ${SOURCES})

# Set target properties
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${raylib_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} raylib)

# Post-build message
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Build successful: ${PROJECT_NAME}"
)