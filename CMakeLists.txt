cmake_minimum_required(VERSION 3.10)

# set the project name
project(semor)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Debug)

# add every executable in a folder
add_executable(semor semor.c src/client.c)


# add include dir
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(semor pthread)