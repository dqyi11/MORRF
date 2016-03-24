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

# Find package FLANN
find_package(PkgConfig)
pkg_check_modules(PC_FLANN flann)
if( PC_FLANN_FOUND )
  message( "FLANN found\n" )
  #add_definitions( -DUSE_FLANN )
endif()

set(FLANN_DEFINITIONS ${PC_FLANN_CFLAGS_OTHER})

find_path(FLANN_INCLUDE_DIR flann/flann.hpp
    HINTS ${PC_FLANN_INCLUDEDIR} ${PC_FLANN_INCLUDE_DIRS})

find_library(FLANN_LIBRARY flann
    HINTS ${PC_FLANN_LIBDIR} ${PC_FLANN_LIBRARY_DIRS})
include_directories(${FLANN_INCLUDE_DIR})


# add the include directories to the build
include_directories(${PROJECT_SOURCE_DIR}/src/MORRF
                    ${PROJECT_SOURCE_DIR}/src/Util
                    ${PROJECT_SOURCE_DIR}/src/MORRFViz
                    ${PROJECT_SOURCE_DIR}/src/MORRFVizDemo
                   )

add_definitions(-Wall -g -O3)

add_subdirectory(MORRF)
add_subdirectory(Util)
add_subdirectory(MORRFViz)
add_subdirectory(MORRFVizDemo)


