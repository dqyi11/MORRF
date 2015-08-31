
set(LIB morrf-viz)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(${LIB} SHARED
            multiobjpathplanninginfo.h
            multiobjpathplanninginfo.cpp
            morrfvisualizer.h
            morrfvisualizer.cpp
           )

target_link_libraries(${LIB} morrf ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
