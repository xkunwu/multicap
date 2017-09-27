if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt.")
endif()

cmake_minimum_required(VERSION 2.8)
SET (CMAKE_CONFIGURATION_TYPES "Debug;Release;RelWithDebInfo;" CACHE STRING "possible configurations" FORCE)
SET (CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Default build type" FORCE)

################################################################################
## project info
################################################################################
project(multicap C CXX)

################################################################################
## output directiory
################################################################################
SET (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/stage/${CMAKE_BUILD_TYPE}")
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/stage/${CMAKE_BUILD_TYPE}")
SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/stage/${CMAKE_BUILD_TYPE}")
SET (X_RES_DIR "${CMAKE_BINARY_DIR}/stage/resources")
SET (X_CFG_DIR "${CMAKE_BINARY_DIR}/stage/config")
SET (X_DOC_DIR "${CMAKE_BINARY_DIR}/stage/docs")
SET (X_LOG_DIR "${CMAKE_BINARY_DIR}/stage/log")

################################################################################
## project directory management
################################################################################
SET_PROPERTY (GLOBAL PROPERTY USE_FOLDERS ON)
include(${CMAKE_SOURCE_DIR}/cmake/utils.cmake)

################################################################################
## global definitions
################################################################################
IF (WIN32)
	SET (CMAKE_CXX_FLAGS "/D_WINDOWS /W3 /GR /EHsc" CACHE STRING "removed WIN32" FORCE)
	SET (CMAKE_C_FLAGS "/D_WINDOWS /W3" CACHE STRING "removed WIN32" FORCE)
	ADD_DEFINITIONS (-DUNICODE -D_UNICODE)
	#ADD_DEFINITIONS(-DNOMINMAX)
	#ADD_DEFINITIONS(-DWIN32_LEAN_AND_MEAN)

	IF (CMAKE_CL_64)
		SET (PlatformName "x64")
		ADD_DEFINITIONS (-DWIN64 -D_WIN64)
	ELSE ()
		SET (PlatformName "win32")
		ADD_DEFINITIONS (-DWIN32 -D_WIN32)
	ENDIF ()

ELSEIF (UNIX AND NOT APPLE)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
	SET(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
ENDIF ()

################################################################################
## 3rd party library
################################################################################
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

find_package(OpenGL REQUIRED)
find_package(GLFW3 REQUIRED)
include_directories(${GLFW3_INCLUDE_DIRS})

add_subdirectory(external/libfreenect2)
set(LIBFREENECT2_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/external/libfreenect2/include
    ${CMAKE_BINARY_DIR}/external/libfreenect2/
)

find_package(Boost 1.45.0 COMPONENTS system filesystem REQUIRED)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(PCL REQUIRED COMPONENTS common io)
set(PCL_LIBRARIES_MIN
    /usr/lib/x86_64-linux-gnu/libpcl_common.so
    /usr/lib/x86_64-linux-gnu/libpcl_octree.so
    /usr/lib/x86_64-linux-gnu/libpcl_io.so
    /usr/lib/x86_64-linux-gnu/libpcl_search.so
    /usr/lib/x86_64-linux-gnu/libpcl_filters.so
    /usr/lib/x86_64-linux-gnu/libpcl_features.so
)

################################################################################
## Add the projects
################################################################################
add_subdirectory(calib)
set(CALIB_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/calib/include)
add_subdirectory(reconstruction)
set(RECON_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/reconstruction)
add_subdirectory(test)
