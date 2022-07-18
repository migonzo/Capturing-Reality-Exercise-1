cmake_minimum_required(VERSION 3.1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(WIN32)
	#set(CERES_INCLUDE_DIRS ${3RDPARTY_PATH}/ceres/include ${3RDPARTY_PATH}/glog/include)
	#set(CERES_LIBS optimized ${3RDPARTY_PATH}/ceres/lib_release/ceres_static.lib debug ${3RDPARTY_PATH}/ceres/lib_debug/ceres_static.lib optimized ${3RDPARTY_PATH}/ceres/lib_release/libglog_static.lib debug ${3RDPARTY_PATH}/ceres/lib_debug/libglog_static.lib)
	#set(CERES_FOUND YES)
	#set(PACKAGE_LIB_SUFFIX_DBG "_debug")
	#set(PACKAGE_LIB_SUFFIX_REL "_release")
	#set(CERES_DIR ${3RDPARTY_PATH}/ceres)
	set(EIGEN3_INCLUDE_DIR ${3RDPARTY_PATH}/eigen)
else()
	find_package(Ceres)
endif()

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

add_executable(ex main.cc ply.cc)
target_include_directories(ex PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(ex image)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(ex Threads::Threads)

if(CERES_FOUND)
	target_include_directories(ex PRIVATE ${CERES_INCLUDE_DIRS})
	target_link_libraries(ex ${CERES_LIBS})
endif()