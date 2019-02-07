########################################################################
# Build Soapy SDR support module for AD-SDR Devices
########################################################################
cmake_minimum_required(VERSION 2.8.7)
project(SoapyADSDR CXX)

find_package(SoapySDR "0.4.0" NO_MODULE REQUIRED)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(ADSDR)

if (NOT ADSDR_FOUND)
    message(FATAL_ERROR "AD-SDR development files not found...")
endif ()
message(STATUS "ADSDR_INCLUDE_DIRS - ${ADSDR_INCLUDE_DIRS}")
message(STATUS "ADSDR_LIBRARIES - ${ADSDR_LIBRARIES}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${ADSDR_INCLUDE_DIRS})

#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #Thread support enabled (not the same as -lpthread)
    list(APPEND ADSDR_LIBRARIES -pthread)

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

if (APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
endif(APPLE)

set(OTHER_LIBS "" CACHE STRING "Other libraries")

SOAPY_SDR_MODULE_UTIL(
    TARGET adsdrSupport
    SOURCES
        SoapyADSDR.hpp
        Registration.cpp
        Settings.cpp
        Streaming.cpp
    LIBRARIES
        ${ADSDR_LIBRARIES}
        ${OTHER_LIBS}
)

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Building for version: ${VERSION} / ${LIBVER}")
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")

########################################################################
# uninstall target
########################################################################
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

#only add uninstall target if this is the top project
if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()



