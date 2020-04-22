cmake_minimum_required(VERSION 3.2)

# Project description
project(
    cmake_cpp_project
    VERSION 1.0
    DESCRIPTION "A CMake/C++ template project"
    LANGUAGES CXX)

# Declare the dependencies of this project
#find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package( Threads )

# Process the src and apps directories
add_subdirectory(src)
add_subdirectory(apps)

# Force the generation of a compile_commands.json file to provide autocompletion for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL "" FORCE)
