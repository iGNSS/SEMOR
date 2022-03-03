cmake_minimum_required(VERSION 3.10)

# set the project name
project(semor)

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# add every executable in a folder
file(GLOB LGI_FILES ./loose-gnss-imu/Loose-GNSS-IMU/Loose-GNSS-IMU/*.cpp) # Loose-GNSS-IMU files
add_executable(semor semor.c src/client.c ${LGI_FILES})


# add include dir
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/loose-gnss-imu/Loose-GNSS-IMU/Loose-GNSS-IMU/include)
target_include_directories(semor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/loose-gnss-imu/Loose-GNSS-IMU/Loose-GNSS-IMU/Libraries/eigen)

target_link_libraries(semor pthread)