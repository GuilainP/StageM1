# Get all the C++ files composing the hello_world application
file(GLOB robot_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.c*)

# Create an executable using these files
add_executable(robot ${robot_FILES})# ${sim_FILES})

# Link the hello library to this executable
target_link_libraries(robot PUBLIC src_SHARED)
