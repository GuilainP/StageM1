# Get all the C++ files composing the hello library
file(GLOB src_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.c*)

# Create a shared library using these files
add_library(src_SHARED ${src_FILES})

# Define the include directory for the library
target_include_directories(src_SHARED PUBLIC ${CMAKE_SOURCE_DIR}/include/
                            PUBLIC ${CMAKE_SOURCE_DIR}/include/sim)

# Set the C++ standard to use for this library

target_compile_features(src_SHARED PUBLIC cxx_std_17)

# Link with the fmt library
target_link_libraries( src_SHARED ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})