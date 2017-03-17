# FindFOVE
# Finds the FOVE Library and include files
#
# This module defines the following variables:
#
# FOVE_FOUND : True if the SDK was found, false otherwise.
# FOVE_INCLUDE_DIRS : Include directories needed to find SQLite headers.
# FOVE_LIBRARIES : The list of all SQLite libraries.
#
# Cache Variables
#
# This module uses the following cache variables:
#
# FOVE_LIBRARY : The location of the FOVE shared library.
# FOVE_INCLUDE_DIR : The location of the FOVE include directory .
#
# The cache variables should not be used by project code.
#
# Author:
# 2017 Georgiy Frolov <georgiy@sensics.com>
#
# Copyright Sensics 2017.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

include(FindPackageHandleStandardArgs)

find_library(FOVE_LIBRARY
    NAMES
    "FoveClient")
    
find_path(FOVE_INCLUDE_DIR
        NAMES
        "IFVRHeadset.h")
    
find_package_handle_standard_args(FOVE FOUND_VAR FOVE_FOUND
    REQUIRED_VARS
    FOVE_INCLUDE_DIR
    FOVE_LIBRARY)
    
if(FOVE_FOUND)
    SET(FOVE_LIBRARIES 
        ${FOVE_LIBRARY})
    SET(FOVE_INCLUDE_DIRS 
        ${FOVE_INCLUDE_DIR})
endif()

mark_as_advanced(FOVE_INCLUDE_DIR FOVE_LIBRARY)