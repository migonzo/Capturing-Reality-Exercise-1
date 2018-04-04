project(CRFramework)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w") # :)

if(CMAKE_VERSION VERSION_LESS "3.1")
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set(CMAKE_CXX_FLAGS "--std=gnu++14 ${CMAKE_CXX_FLAGS}")
	endif()
else()
	set(CMAKE_CXX_STANDARD 14)
endif()

find_package(PNG)
find_package(JPEG)
find_package(OpenEXR)
find_package (Eigen3 REQUIRED)

if (PNG_FOUND)
add_definitions(-DWITH_PNG)
endif()
if (JPEG_FOUND)
add_definitions(-DWITH_JPEG)
endif()
if (OPENEXR_FOUND)
add_definitions(-DWITH_EXR)
endif()

include_directories(${PNG_INCLUDE_DIRS} ${JPEG_INCLUDE_DIR} ${OPENEXR_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} "${CMAKE_SOURCE_DIR}/include")

add_executable(example src/image_io.cc example.cc)
target_link_libraries(example ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${OPENEXR_LIBRARIES})