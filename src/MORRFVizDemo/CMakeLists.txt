set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(morrf-viz-demo
               configobjdialog.h
               configobjdialog.cpp
               mainwindow.h
               mainwindow.cpp
               main.cpp
               )

include_directories(
                   ${Qt5Widgets_INCLUDE_DIRS}
                   ${Qt5Core_INCLUDE_DIRS}
                   )

target_link_libraries(morrf-viz-demo
                      morrf
                      morrf-viz
                      ${Qt5Widgets_LIBRARIES}
                      ${Qt5Core_LIBRARIES}
                     )
