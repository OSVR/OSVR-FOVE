# FOVE plugin for OSVR
cmake_minimum_required(VERSION 3.1.0)
project(Fove_OSVR_plugin)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find OSVR
# Add default OSVR paths just to increase the chance things work by default
# If OSVR can't be found, the user can point to the osvr/cmake dir from cmake
if (WIN32)
	list(APPEND CMAKE_PREFIX_PATH "C:/Program Files/OSVR/SDK/x64/lib/cmake")
endif()
find_package(osvr REQUIRED)
find_package(FOVE REQUIRED)

osvr_convert_json(com_osvr_fove_eyetracker_json
	com_osvr_fove_eyetracker.json
	"${CMAKE_CURRENT_BINARY_DIR}/com_osvr_fove_eyetracker_json.h")

osvr_convert_json(com_osvr_fove_tracker_json
	com_osvr_fove_tracker.json
	"${CMAKE_CURRENT_BINARY_DIR}/com_osvr_fove_tracker_json.h")


# Generate OSVR plugin DLL
osvr_add_plugin(NAME com_Sensics_FOVE SOURCES
	com_osvr_fove.cpp
	"${CMAKE_CURRENT_BINARY_DIR}/com_osvr_fove_eyetracker_json.h"
	"${CMAKE_CURRENT_BINARY_DIR}/com_osvr_fove_tracker_json.h"
)

target_link_libraries(com_Sensics_FOVE
	osvr::osvrUtilCpp
	osvr::osvrPluginKitCpp
	${FOVE_LIBRARIES})

target_include_directories(com_Sensics_FOVE
	PUBLIC
	${FOVE_INCLUDE_DIRS}
	PRIVATE
	${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)
if(NOT CMAKE_INSTALL_DOCDIR)
	set(CMAKE_INSTALL_DOCDIR ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME})
endif()

install(FILES
	README.md
	LICENSE
	DESTINATION
	${CMAKE_INSTALL_DOCDIR})

install(FILES
	displays/FOVE_0.json
	DESTINATION
	${OSVR_CONFIG_ROOT}/displays)

