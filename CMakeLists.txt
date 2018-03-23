project(CRFramework)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenEXR REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories(${PNG_INCLUDE_DIRS} ${JPEG_INCLUDE_DIR} ${OPENEXR_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(test image.cc test.cc)
target_link_libraries(test ${PNG_LIBRARIES} ${JPEG_LIBRARIES} ${OPENEXR_LIBRARIES})