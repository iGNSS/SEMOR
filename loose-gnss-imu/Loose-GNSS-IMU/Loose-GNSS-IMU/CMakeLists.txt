cmake_minimum_required(VERSION 3.10)

# set the project name
project(LGI)

#add_compile_options(-Wconversion-null)

# add every executable in a folder
file(GLOB SRC_FILES ./*.cpp)
add_executable(LGI ${SRC_FILES})


# add include dir
target_include_directories(LGI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(LGI PUBLIC Libraries/eigen)