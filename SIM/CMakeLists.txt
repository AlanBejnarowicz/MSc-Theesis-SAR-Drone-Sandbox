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

# Set source directory root
set(SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Include ALL necessary directories
include_directories(
    ${SRC_ROOT}
    ${SRC_ROOT}/Tools
    ${SRC_ROOT}/Objects
)

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

# Find ALL source files
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/*.c"
    "src/Tools/*.cpp"
    "src/Tools/*.c"
    "src/Objects/*.cpp"
    "src/Objects/*.c"
    "src/Objects/**/*.cpp"
    "src/Objects/**/*.c"
)

# Also find header files (optional, for IDE support)
file(GLOB_RECURSE HEADERS
    "src/*.h"
    "src/*.hpp"
    "src/Tools/*.h"
    "src/Tools/*.hpp"
    "src/Objects/*.h"
    "src/Objects/*.hpp"
    "src/Objects/**/*.h"
    "src/Objects/**/*.hpp"
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Copy resource files to build directory
# file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/rsc DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Remove the old file(COPY ...) line and use this:
if(WIN32)
    # Windows version (creates a junction/link)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink 
        "${CMAKE_CURRENT_SOURCE_DIR}/rsc" 
        "${CMAKE_CURRENT_BINARY_DIR}/rsc"
    )
else()
    # Linux/Mac version
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E create_symlink 
        "${CMAKE_CURRENT_SOURCE_DIR}/rsc" 
        "${CMAKE_CURRENT_BINARY_DIR}/rsc"
    )
endif()


# Set target properties - FIXED ORDER
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${SRC_ROOT}
    ${SRC_ROOT}/Tools
    ${SRC_ROOT}/Objects
)

# Link raylib
target_link_libraries(${PROJECT_NAME} raylib)

# Post-build message
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Build successful: ${PROJECT_NAME}"
)