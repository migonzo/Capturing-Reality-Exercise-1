cmake_minimum_required(VERSION 3.1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(WIN32)
	set(EIGEN3_INCLUDE_DIR ${3RDPARTY_PATH}/eigen)
endif()

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(Ceres)

add_executable(ex main.cc)
target_include_directories(ex PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ex image)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(ex Threads::Threads)

if(CERES_FOUND)
	target_include_directories(ex PRIVATE ${CERES_INCLUDE_DIRS})
	message(STATUS ${CERES_LIBS})
	target_link_libraries(ex ${CERES_LIBS})
endif()