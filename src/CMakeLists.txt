set(CMAKE_BUILD_WITH_INSTALL_PATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x)

# search for the QT Package, exit if not found
find_package(Qt COMPONENTS QtCore QtGUI REQUIRED)
if( NOT QT_FOUND )
  message( FATAL_ERROR "Qt not found\n")
  return()
endif()
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIR})

# add the include directories to the build
include_directories(${PROJECT_SOURCE_DIR}/src/MORRF
                    ${PROJECT_SOURCE_DIR}/src/Util
                   # ${PROJECT_SOURCE_DIR}/src/MORRFViz
                   # ${PROJECT_SOURCE_DIR}/src/MORRFVizDemo
                   )
