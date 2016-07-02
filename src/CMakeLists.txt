set(CMAKE_BUILD_WITH_INSTALL_PATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x)

find_package(Qt5Core)
if( NOT Qt5Core_FOUND )
  message( FATAL_ERROR "Qt5Core not found\n" )
  return()
endif()

find_package(Qt5Widgets)
if( NOT Qt5Widgets_FOUND )
  message( FATAL_ERROR "Qt5Widgets not found\n" )
  return()
endif()

# search for the getgetopt program, exit if not found
find_program(GENGETOPT gengetopt)
if( NOT GENGETOPT )
  message( FATAL_ERROR "gengetopt not found\n" )
  return()
endif()

# function to generate the getgetopt source and header from a .ggo file
function(generate_ggo)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  cmake_parse_arguments(GENERATE_GGO "" "INPUT;OUTPUT" "" ${ARGN})
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${GENERATE_GGO_OUTPUT}.c ${CMAKE_CURRENT_BINARY_DIR}/${GENERATE_GGO_OUTPUT}.h
    COMMAND ${GENGETOPT} -i ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_GGO_INPUT} --file-name ${GENERATE_GGO_OUTPUT} --output-dir=${CMAKE_CURRENT_BINARY_DIR} --unamed-opts
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATE_GGO_INPUT}
  )
endfunction(generate_ggo)

# Find package FLANN
find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
#if( PC_FLANN_FOUND )
#  message( "FLANN found\n" )
#endif()

set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS})

find_library(FLANN_LIBRARY flann
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})
include_directories(${FLANN_INCLUDE_DIR})


# add the include directories to the build
include_directories(${PROJECT_SOURCE_DIR}/src/morrf
                    ${PROJECT_SOURCE_DIR}/src/util
                    ${PROJECT_SOURCE_DIR}/src/morrf_viz
                    ${PROJECT_SOURCE_DIR}/src/morrf_viz_demo
                   )

add_definitions(-Wall -g -O3)

add_subdirectory(morrf)
add_subdirectory(util)
add_subdirectory(morrf_viz)
add_subdirectory(morrf_viz_demo)


