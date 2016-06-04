
set(LIB morrf)

set(${HDR} morrf/kdtree++/allocator.hpp
           morrf/kdtree++/function.hpp
           morrf/kdtree++/iterator.hpp
           morrf/kdtree++/kdtree.hpp
           morrf/kdtree++/node.hpp
           morrf/kdtree++/region.hpp
           morrf/KDTree2D.h
           morrf/utilities.h
           morrf/subtree.h
           morrf/morrf.h
    )


add_library(${LIB} SHARED
            ${HDR}
            subtree.cpp
            morrf.cpp
           )

target_link_libraries(${LIB})

install(TARGETS ${LIB}
        LIBRARY
        DESTINATION lib
        ARCHIVE
        DESTINATION lib)

install(DIRECTORY morrf DESTINATION include)
