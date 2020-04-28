## V-REP remote API library
set(VREP_DEFINITIONS NON_MATLAB_PARSING MAX_EXT_API_CONNECTIONS=255)
if(APPLE)
    list(APPEND VREP_DEFINITIONS __APPLE__)
elseif(UNIX)
    list(APPEND VREP_DEFINITIONS __linux)
elseif(WIN32)
    list(APPEND VREP_DEFINITIONS __WIN32)
endif()

file(GLOB sim_FILES ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.c)
add_library(sim SHARED ${sim_FILES})
target_compile_definitions(sim PUBLIC ${VREP_DEFINITIONS})
target_include_directories(sim PUBLIC ${CMAKE_SOURCE_DIR}/include/sim)

# E-Puck library
file(GLOB epuck_FILES ${CMAKE_CURRENT_SOURCE_DIR}/epuck/*.cpp)
add_library(epuck SHARED ${epuck_FILES})
target_include_directories(epuck PUBLIC ${CMAKE_SOURCE_DIR}/include/epuck)
target_compile_features(epuck PUBLIC cxx_std_17)
target_link_libraries(epuck sim ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})