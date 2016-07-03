set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

generate_ggo(morrf_viz_demo.ggo morrf_viz_demo_cmdline)

add_executable(morrf-viz-demo
               configobjdialog.h
               configobjdialog.cpp
               mainwindow.h
               mainwindow.cpp
               morrf_viz_demo.cpp
               ${CMAKE_CURRENT_BINARY_DIR}/morrf_viz_demo_cmdline.c
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
