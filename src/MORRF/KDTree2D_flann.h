#ifndef KDTREE2D_FLANN_H
#define KDTREE2D_FLANN_H

#include <flann/flann.hpp>
#include <list>
#include <functional>
#include <iostream>

class RRTNode;

class POS2D : public flann::Matrix<int>{
public:
    typedef int value_type;

    POS2D( value_type x = 0, value_type y = 0) {
        d[0] = x;
        d[1] = y;
        flann::Matrix<int>(d, 1, 2);
    }

    POS2D( const POS2D & x ) {
        d[0] = x.d[0];
        d[1] = x.d[1];
        flann::Matrix<int>(x.ptr(), 1, 2);
    }

    ~POS2D(){}

    double distance_to(POS2D const& x) const  {
        double dist = 0;
        for ( int i = 0; i != 2; ++i) {
            dist += (d[i]-x.d[i])*(d[i]-x.d[i]);
        }
        return std::sqrt(dist);
    }

    value_type operator[]( size_t const N ) { return d[N]; }

    void setX( value_type x ) {
        d[0] = x;
        flann::Matrix<int>(d, 1, 2);
    }
    void setY( value_type y ) {
        d[1] = y;
        flann::Matrix<int>(d, 1, 2);
    }

    bool operator==(const POS2D &other) const {
        return d[0] == other.d[0] && d[1] == other.d[1];
    }

    value_type d[2];
};

class KDNode2D : public POS2D {
public:
    KDNode2D( value_type x, value_type y ) : POS2D( x, y ) { m_node_list.clear(); }
    KDNode2D( POS2D & pos ) : POS2D( pos ) { m_node_list.clear(); }

    std::vector<RRTNode*> m_node_list;
};


inline std::ostream& operator<< ( std::ostream& out, POS2D const& T ) {
    return out << '(' << T.d[0] << ',' << T.d[1] << ')';
}


inline std::ostream& operator<< ( std::ostream& out, KDNode2D const& T ) {
    return out << '(' << T.d[0] << ',' << T.d[1] << ')';
}

//inline double tac( KDNode2D t, size_t k ) { return t[k]; }

//typedef KDTree::KDTree< 2, KDNode2D, std::pointer_to_binary_function< KDNode2D, size_t, double > > KDTree2D;

class KDTree2D {
public:
    KDTree2D() {
        _p_index = NULL;
    }

    virtual ~KDTree2D() {
        if(_p_index) {
            delete _p_index;
            _p_index = NULL;
        }
    }

    void insert( KDNode2D node ) {
        if( _p_index == NULL ) {
            _p_index = new flann::Index<flann::L2<int> >( node, flann::KDTreeIndexParams(4) );
        }
        else {
            _p_index->addPoints( node );
        }
    }

    bool find( KDNode2D node ) {
        return true;
    }

    size_t size() const {
        return _p_index->size();
    }

    void find_within_range( POS2D pos, float range, std::list<KDNode2D>& node_list ) {
        if( _p_index ) {
            std::vector<std::vector<int> > indices;
            std::vector<std::vector<float> > dists;
            _p_index->radiusSearch( pos , indices, dists, range, flann::SearchParams(128));
            for(unsigned int i=0;i<indices.size();i++) {

            }
        }
    }

    KDNode2D* find_nearest( POS2D pos ) {
        if( _p_index ) {
            std::vector<std::vector<int> > indices;
            std::vector<std::vector<float> > dists;
            _p_index->knnSearch( pos , indices, dists, 1, flann::SearchParams(128));
        }
        return NULL;
    }

    flann::Index<flann::L2<int> >* _p_index;
};


#endif // KDTREE2D_FLANN_H
