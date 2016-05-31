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


