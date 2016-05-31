
set(LIB morrf-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            morrf_viz/multiobjpathplanninginfo.h
            morrf_viz/morrfvisualizer.h
            multiobjpathplanninginfo.cpp
            morrfvisualizer.cpp
           )

include_directories(
                    ${Qt5Widgets_INCLUDE_DIRS}
                   )

target_link_libraries(${LIB} 
                      morrf
                      ${Qt5Widgets_LIBRARIES}
                     )
