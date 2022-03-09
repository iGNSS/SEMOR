cmake_minimum_required(VERSION 3.10)

# set the project name
project(semor)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Debug)

# add every executable in a folder
file(GLOB LGI_FILES ./loose-gnss-imu/*.cpp) # Loose-GNSS-IMU files
add_executable(semor semor.c src/client.c src/ctocpp.cpp ${LGI_FILES})


# add include dir
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/loose-gnss-imu/include)
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/loose-gnss-imu/Libraries/eigen)

target_link_libraries(semor pthread)