
set(LIB morrf)

set(${HDR} include/morrf/kdtree++/allocator.hpp
           include/morrf/kdtree++/function.hpp
           include/morrf/kdtree++/iterator.hpp
           include/morrf/kdtree++/kdtree.hpp
           include/morrf/kdtree++/node.hpp
           include/morrf/kdtree++/region.hpp
           include/morrf/KDTree2D.h
           include/morrf/utilities.h
           include/morrf/subtree.h
           include/morrf/morrf.h
    )

include_directories(
                   include
                   )

add_library(${LIB} SHARED
            ${HDR}
            subtree.cpp
            morrf.cpp
           )

target_link_libraries(${LIB})
