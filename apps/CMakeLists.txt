# Robot control application
file(GLOB robot_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(robot ${robot_FILES})
target_link_libraries(robot PUBLIC epuck)
