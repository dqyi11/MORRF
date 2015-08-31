set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(configobjdialog.h
               configobjdialog.cpp
               mainwindow.h
               mainwindow.cpp
               main.cpp
               )

target_link_libraries(morrf
                      morrf-viz
                      ${QT_QTCORE_LIBRARY}
                      ${QT_QTGUI_LIBRARY}
                     )
