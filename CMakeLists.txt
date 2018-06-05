project(CRFramework)

cmake_minimum_required(VERSION 3.1)

if(WIN32)
	set(3RDPARTY_PATH "${CMAKE_SOURCE_DIR}/3rdparty" CACHE PATH "Location of the 3rd party libraries")
	if(NOT EXISTS "${3RDPARTY_PATH}/")
		message(FATAL_ERROR "Could not fild the '3rdparty' folder. Please download the 3rd party libraries from moodle and extract them to '${3RDPARTY_PATH}'.")
	endif()
endif()

add_subdirectory(image)
add_subdirectory(exercise)