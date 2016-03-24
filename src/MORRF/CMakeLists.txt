
set(LIB morrf)

add_library(${LIB} SHARED
            kdtree++/allocator.hpp
            kdtree++/function.hpp
            kdtree++/iterator.hpp
            kdtree++/kdtree.hpp
            kdtree++/node.hpp
            kdtree++/region.hpp
            KDTree2D.h
            KDTree2D_flann.h
            utilities.h
            subtree.h
            subtree.cpp
            morrf.h
            morrf.cpp
           )

target_link_libraries(${LIB})
