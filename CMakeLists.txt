# FOVE plugin for OSVR
cmake_minimum_required(VERSION 3.1.0)
project(Fove_OSVR_plugin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 14)
add_definitions(-DUNICODE)


# Find OSVR
# Add default OSVR paths just to increase the chance things work by default
# If OSVR can't be found, the user can point to the osvr/cmake dir from cmake
if (WIN32)
	list(APPEND CMAKE_PREFIX_PATH "C:/Program Files/OSVR/SDK/x64/lib/cmake")
endif()
find_package(osvr REQUIRED)
find_package(FOVE REQUIRED)

osvr_convert_json(com_osvr_FOVE_eye_tracking_config_json
    eye_tracking_config.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FOVE_eye_tracking_config_json.h")
	
osvr_convert_json(com_osvr_FOVE_head_tracking_config_json
    head_tracking_config.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FOVE_head_tracking_config_json.h")

include_directories("${CMAKE_CURRENT_BINARY_DIR}")
	
# Generate OSVR plugin DLL
osvr_add_plugin(NAME com_Sensics_FOVE SOURCES
	com_osvr_fove.cpp
	com_osvr_fove.h
	"${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FOVE_eye_tracking_config_json.h"
	"${CMAKE_CURRENT_BINARY_DIR}/com_osvr_FOVE_head_tracking_config_json.h"
)
# set_target_properties(com_Sensics_FOVE 
						# PROPERTIES 
						# PROJECT_LABEL "Fove OSVR Plugin")

target_link_libraries(com_Sensics_FOVE 
						osvr::osvrUtilCpp
						osvr::osvrPluginKitCpp
						${FOVE_LIBRARIES})

target_include_directories(com_Sensics_FOVE
							PUBLIC
							${FOVE_INCLUDE_DIRS})