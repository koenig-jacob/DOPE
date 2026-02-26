cmake_minimum_required(VERSION 3.10)

project(DOPE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(lib/bce/include)

# Source files
file(GLOB_RECURSE BCE_SOURCES
    lib/bce/src/*.cpp
)

# Create the library
add_library(dope STATIC ${BCE_SOURCES})

# Add executable for GUI
add_executable(gui src/gui_main.cpp src/imgui_harness.cpp)

# Link the DOPE library to the GUI executable
target_link_libraries(gui dope)

# Enable testing
enable_testing()

# Add tests
file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} dope)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()