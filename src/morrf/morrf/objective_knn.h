#ifndef OBJECTIVE_KNN_H_
#define OBJECTIVE_KNN_H_

#include <vector>
#include <limits>
#include <flann/flann.hpp>


class ObjectiveKNN {
public:
    ObjectiveKNN(int Knn, flann::Matrix<float>& vecs) {
        m_Knn = Knn;
        _p_index = new flann::Index<flann::L2<float> >( vecs, flann::KDTreeIndexParams(4) );
        _p_index->buildIndex();
    }

    virtual ~ObjectiveKNN() {
        if(_p_index) {
            delete _p_index;
            _p_index = NULL;
        }
    }

    std::vector<float> get_sparse_diversity( flann::Matrix<float>& vecs ) {

        std::vector<float> sparse_diversity( vecs.rows, 0.0 );
        if( _p_index ) {
            std::vector<std::vector<int> > indices;
            std::vector<std::vector<float> > dists;
            _p_index->knnSearch( vecs , indices, dists, m_Knn, flann::SearchParams(128));

            //std::cout << "indices size " << indices.size() << std::endl;
            for(unsigned int i=0;i<indices.size();i++) {
                //std::cout << "indices[" << i << "] size " << indices[i].size() << std::endl;
                for(unsigned int j=0;j<indices[i].size();j++) {
                    sparse_diversity[i] += dists[i][j];
                }
            }
        }
        return sparse_diversity;
    }

    int m_Knn;
    flann::Index<flann::L2<float> >* _p_index;
};


#endif // OBJECTIVE_KNN_H_
