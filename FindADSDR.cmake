#
# Copyright 2012-2013 The Iris Project Developers. See the
# COPYRIGHT file at the top-level directory of this distribution
# and at http://www.softwareradiosystems.com/iris/copyright.html.
#
# This file is part of the Iris Project.
#
# Iris is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# Iris is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# A copy of the GNU Lesser General Public License can be found in
# the LICENSE file in the top-level directory of this distribution
# and at http://www.gnu.org/licenses/.
#

# - Try to find adsdr - the hardware driver for the AD chip in Amungo receiver
# Once done this will define
#  ADSDR_FOUND - System has adsdr
#  ADSDR_LIBRARIES - The adsdr libraries
#  ADSDR_INCLUDE_DIRS - The adsdr include directories
#  ADSDR_LIB_DIRS - The adsdr library directories

if(NOT ADSDR_FOUND)

    find_package(PkgConfig)
    pkg_check_modules (ADSDR_PKG libadsdr)
    set(ADSDR_DEFINITIONS ${PC_ADSDR_CFLAGS_OTHER})

    find_path(ADSDR_INCLUDE_DIR
                NAMES adsdr.h
                HINTS ${ADSDR_PKG_INCLUDE_DIRS} $ENV{ADSDR_DIR}/include
                PATHS /usr/local/include /usr/include /opt/include /opt/local/include)

    find_library(ADSDR_LIBRARY
                NAMES adsdr
                HINTS ${ADSDR_PKG_LIBRARY_DIRS} $ENV{ADSDR_DIR}/include
                PATHS /usr/local/lib /usr/lib /opt/lib /opt/local/lib)

    set(ADSDR_LIBRARIES ${ADSDR_LIBRARY} )
    set(ADSDR_INCLUDE_DIRS ${ADSDR_INCLUDE_DIR} )

    include(FindPackageHandleStandardArgs)
    # handle the QUIETLY and REQUIRED arguments and set LibADSDR_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(adsdr  DEFAULT_MSG
                                      ADSDR_LIBRARY ADSDR_INCLUDE_DIR)

    mark_as_advanced(ADSDR_INCLUDE_DIR ADSDR_LIBRARY)

endif(NOT ADSDR_FOUND)

