cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(WIN32)
	set(EIGEN3_INCLUDE_DIR ${3RDPARTY_PATH}/eigen)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") # :)

find_package(Eigen3 REQUIRED)

add_executable(ex main.cc)
target_include_directories(ex PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ex image)