#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include "morrf.h"

#define OBSTACLE_THRESHOLD 200

MORRF::MORRF(int width, int height, int objective_num, int subproblem_num, int segmentLength, MORRF_TYPE type) {
    _sampling_width = width;
    _sampling_height = height;
    _objective_num = objective_num;
    _subproblem_num = subproblem_num;
    _type = type;

    _p_kd_tree = new KDTree2D( std::ptr_fun(tac) );

    _range = ( _sampling_width > _sampling_height ) ? _sampling_width:_sampling_height;
    _ball_radius = _range;
    _obs_check_resolution = 1;
    _current_iteration = 0;
    _segment_length = segmentLength;

    _pp_weights = NULL;
    _theta = 5;

    _pp_map_info = new int*[_sampling_width];
    for( int i=0; i<_sampling_width; i++ ) {
        _pp_map_info[i] = new int[_sampling_height];
        for( int j=0; j<_sampling_height; j++) {
            _pp_map_info[i][j] = 255;
        }
    }
}

MORRF::~MORRF() {
    _deinit_weights();

    if( _p_kd_tree ) {
        delete _p_kd_tree;
        _p_kd_tree = NULL;
    }
}

void MORRF::add_funcs( std::vector<COST_FUNC_PTR> funcs, std::vector<int**> fitnessDistributions) {
    _funcs = funcs;
    _fitness_distributions = fitnessDistributions;
}

void MORRF::_init_weights() {
    _deinit_weights();

    _pp_weights = new double*[_subproblem_num];

    if ( _objective_num == 2 ) {
        for( int i=0; i<_subproblem_num; i++ ) {
            _pp_weights[i] = new double[_objective_num];
            _pp_weights[i][0] = (double)(i+1) / (double)(_subproblem_num+2);
            _pp_weights[i][1] = (double)(_subproblem_num-i+1) / (double)(_subproblem_num+2);
        }
    }
    else {
        for( int i=0; i<_subproblem_num; i++ ) {
            _pp_weights[i] = new double[_objective_num];
            for( int j=0; j<_objective_num; j++) {
                _pp_weights[i][j] = (double)rand()/RAND_MAX;
            }
        }
    }
}

void MORRF::_deinit_weights() {
    if( _pp_weights ) {
        int size = sizeof(_pp_weights)/sizeof(double*);
        for( int i=0; i<size; i++ ) {
            if( _pp_weights[i] ) {
                delete _pp_weights[i];
                _pp_weights[i] = NULL;
            }
        }
    }
}

void MORRF::init(POS2D start, POS2D goal) {

    _init_weights();

    KDNode2D root(start);
    for( int k=0; k<_objective_num; k++ ) {
        ReferenceTree * p_ref_tree = new ReferenceTree( this, _objective_num, k );
        RRTNode * p_root_node = p_ref_tree->init( start, goal );
        root.m_node_list.push_back( p_root_node );
        _references.push_back( p_ref_tree );
    }

    for( int m=0; m<_subproblem_num; m++ ) {
        SubproblemTree * p_sub_tree = new SubproblemTree( this, _objective_num, _pp_weights[m], m );
        RRTNode * p_root_node = p_sub_tree->init( start, goal );
        root.m_node_list.push_back(p_root_node);
        _subproblems.push_back( p_sub_tree );
    }
    _p_kd_tree->insert( root );
    _current_iteration = 0;
}

void MORRF::load_map(int **map) {
    for( int i=0; i<_sampling_width; i++ ) {
        for( int j=0; j<_sampling_height; j++ ) {
            _pp_map_info[i][j] = map[i][j];
        }
    }
}

POS2D MORRF::sampling() {
    double x = rand();
    double y = rand();
    x = x * ((double)(_sampling_width)/RAND_MAX);
    y = y * ((double)(_sampling_height)/RAND_MAX);

    POS2D m(x,y);
    return m;
}

POS2D MORRF::steer( POS2D pos_a, POS2D pos_b ) {
    POS2D new_pos( pos_a[0], pos_a[1] );
    double delta[2];
    delta[0] = pos_a[0] - pos_b[0];
    delta[1] = pos_a[1] - pos_b[1];
    double delta_len = std::sqrt( delta[0]*delta[0]+delta[1]*delta[1] );

    if ( delta_len > _segment_length ) {
        double scale = _segment_length / delta_len;
        delta[0] = delta[0] * scale;
        delta[1] = delta[1] * scale;

        new_pos.setX( pos_b[0]+delta[0] );
        new_pos.setY( pos_b[1]+delta[1] );
    }
    return new_pos;
}

bool MORRF::_is_in_obstacle( POS2D pos ) {
    int x = (int)pos[0];
    int y = (int)pos[1];
    if( _pp_map_info[x][y] < OBSTACLE_THRESHOLD )
        return true;
    return false;
}


bool MORRF::_is_obstacle_free( POS2D pos_a, POS2D pos_b ) {
    if ( pos_a == pos_b ) {
        return true;
    }
    int x_dist = pos_a[0] - pos_b[0];
    int y_dist = pos_a[1] - pos_b[1];

    if( x_dist == 0 && y_dist == 0 ) {
        return true;
    }

    float x1 = pos_a[0];
    float y1 = pos_a[1];
    float x2 = pos_b[0];
    float y2 = pos_b[1];

    const bool steep = ( fabs( y2 - y1 ) > fabs( x2 - x1 ) );
    if ( steep ) {
        std::swap( x1, y1 );
        std::swap( x2, y2 );
    }

    if ( x1 > x2 ) {
        std::swap( x1, x2 );
        std::swap( y1, y2 );
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = ( y1 < y2 ) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for( int x=(int)x1; x<maxX; x++ ) {
        if( steep ) {
            if ( y >= 0 && y < _sampling_width && x >= 0 && x < _sampling_height ) {
                if ( _pp_map_info[y][x] < OBSTACLE_THRESHOLD ) {
                    return false;
                }
            }
        }
        else {
            if ( x >= 0 && x < _sampling_width && y >= 0 && y < _sampling_height ) {
                if ( _pp_map_info[x][y] < OBSTACLE_THRESHOLD ) {
                    return false;
                }
            }
        }

        error -= dy;
        if( error < 0 ) {
            y += ystep;
            error += dx;
        }
    }

    return true;
}

void MORRF::extend() {

    bool node_inserted = false;
    while( false == node_inserted ) {
        POS2D rndPos = sampling();
        KDNode2D nearest_node = find_nearest( rndPos );

        POS2D new_pos = steer( rndPos, nearest_node );

        if(true == _contains( new_pos )) {
            continue;
        }
        if( true == _is_in_obstacle( new_pos ) ) {
            continue;
        }

        if( true == _is_obstacle_free( nearest_node, new_pos )) {
            std::list<KDNode2D> near_nodes = find_near( new_pos );
            KDNode2D new_node( new_pos );

            // create new nodes of reference trees
            for( int k=0; k < _objective_num; k++ ) {
                RRTNode * p_new_ref_node = _references[k]->create_new_node( new_pos );
                new_node.m_node_list.push_back( p_new_ref_node );
            }

            // create new nodes of subproblem trees
            for ( int m=0; m < _subproblem_num; m++ ) {
                RRTNode * p_new_sub_node = _subproblems[m]->create_new_node( new_pos );
                new_node.m_node_list.push_back( p_new_sub_node );
            }

            _p_kd_tree->insert(new_node);
            node_inserted = true;

            // attach new node to reference trees
            // rewire near nodes of reference trees
            for ( int k=0; k<_objective_num; k++ ) {
                // std::cout << "@ " << k << std::endl;
                int index = k;
                RRTNode* p_nearest_ref_node = nearest_node.m_node_list[index];
                RRTNode* p_new_ref_node = new_node.m_node_list[index];
                std::list<RRTNode*> near_ref_nodes;
                near_ref_nodes.clear();
                for( std::list<KDNode2D>::iterator itr = near_nodes.begin();
                    itr != near_nodes.end(); itr++) {
                    KDNode2D kd_node = (*itr);
                    RRTNode* pRefNode = kd_node.m_node_list[index];
                    near_ref_nodes.push_back( pRefNode );
                }

                _references[k]->attach_new_node( p_new_ref_node, p_nearest_ref_node, near_ref_nodes );
                _references[k]->rewire_near_nodes( new_node.m_node_list[index], near_ref_nodes );
            }

            // attach new nodes to subproblem trees
            // rewire near nodes of subproblem trees
            for( int m=0; m<_subproblem_num; m++ ) {
                // std::cout << "@ " << m+mObjectiveNum << std::endl;
                int index = m + _objective_num;
                RRTNode* p_nearest_sub_node = nearest_node.m_node_list[index];
                RRTNode* p_new_sub_node = new_node.m_node_list[index];
                std::list<RRTNode*> near_sub_nodes;
                near_sub_nodes.clear();
                for( std::list<KDNode2D>::iterator its = near_nodes.begin();
                    its != near_nodes.end(); its++) {
                    KDNode2D kd_node = (*its);
                    RRTNode* p_sub_node = kd_node.m_node_list[index];
                    near_sub_nodes.push_back( p_sub_node );
                }

                _subproblems[m]->attach_new_node( p_new_sub_node, p_nearest_sub_node, near_sub_nodes );
                _subproblems[m]->rewire_near_nodes( new_node.m_node_list[index], near_sub_nodes );
            }
        }
    }
    _current_iteration++;
}

KDNode2D MORRF::find_nearest( POS2D pos ) {
    KDNode2D node( pos );

    std::pair<KDTree2D::const_iterator,double> found = _p_kd_tree->find_nearest( node );
    KDNode2D near_node = *found.first;
    return near_node;
}

std::list<KDNode2D> MORRF::find_near( POS2D pos ) {
    std::list<KDNode2D> near_list;
    KDNode2D node(pos);

    int numVertices = _p_kd_tree->size();
    int numDimensions = 2;
    _ball_radius = _theta * _range * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );

    _p_kd_tree->find_within_range( node, _ball_radius, std::back_inserter(near_list) );

    return near_list;
}


bool MORRF::_contains( POS2D pos ) {
    if( _p_kd_tree ) {
        KDNode2D node( pos[0], pos[1] );
        KDTree2D::const_iterator it = _p_kd_tree->find( node );
        if( it != _p_kd_tree->end() ) {
            return true;
        }
        else {
            return false;
        }
    }
    return false;
}

bool MORRF::calc_cost(POS2D& pos_a, POS2D& pos_b, double * p_cost) {
    if ( p_cost == NULL ) {
        return false;
    }
    for( int k = 0; k < _objective_num; k++ ) {
        p_cost[k] = calc_cost( pos_a, pos_b, k );
    }
    return true;
}

double MORRF::calc_cost( POS2D& pos_a, POS2D& pos_b, int k ) {
    return _funcs[k]( pos_a, pos_b, _fitness_distributions[k] );
}

double MORRF::calc_fitness( double * p_cost, double * p_weight, POS2D& pos ) {
    double fitness = 0.0;
    if( p_cost == NULL || p_weight==NULL ) {
        return fitness;
    }
    if( _type==MORRF::WEIGHTED_SUM ) {
        for( int k=0; k<_objective_num; k++ ) {
            fitness += p_cost[k] * p_weight[k];
        }
    }
    else if( _type==MORRF::TCHEBYCHEFF ) {
        double p_utopia[_objective_num];
        if( true == get_utopia_reference_vector( pos, p_utopia ) ) {
            for( int k=0; k<_objective_num; k++ ) {
               double weighted_dist = p_weight[k] * fabs( p_cost[k] - p_utopia[k] );
               if ( weighted_dist > fitness ) {
                   fitness = weighted_dist;
               }
            }
        }
    }
    else {
        double p_utopia[_objective_num];
        if( true == get_utopia_reference_vector( pos, p_utopia ) ) {
            double d1 = 0.0, d2 = 0.0;
            for( int k=0; k<_objective_num; k++ ) {
               double weighted_dist = p_weight[k] * (p_cost[k] - p_utopia[k]);
               d1 += weighted_dist;
            }
            d1 = fabs(d1);
            double vectorD2[_objective_num];
            for( int k=0; k<_objective_num; k++ ) {
                vectorD2[k] = p_cost[k] - (p_utopia[k] + d1* p_weight[k]);
                d2 += vectorD2[k]*vectorD2[k];
            }
            d2 = fabs(sqrt(d2));
            fitness = d1 + _theta * d2;
        }
    }
    if(fitness < 0.0) {
        std::cout << "Negative fitness " << fitness << std::endl;
    }
    return fitness;
}

bool MORRF::get_utopia_reference_vector( POS2D& pos, double * p_utopia ) {
    if ( p_utopia==NULL ) {
        return false;
    }
    KDNode2D ref_node = find_nearest(pos);
    if( ref_node.m_node_list.size()<_objective_num ) {
        return false;
    }

    for( int k=0; k<_objective_num; k++ ) {
        RRTNode* pRRTNode = ref_node.m_node_list[k];
        p_utopia[k] = pRRTNode->m_fitness;
    }
    return true;
}

ReferenceTree* MORRF::get_reference_tree(int k) {
    if( k<0 || k>=_objective_num ) {
        return NULL;
    }
    return _references[k];
}

SubproblemTree* MORRF::get_subproblem_tree( int m ) {
    if( m<0 || m>=_subproblem_num ) {
        return NULL;
    }
    return _subproblems[m];
}

void MORRF::dump_map_info( std::string filename ) {
    std::ofstream mapInfoFile;
    mapInfoFile.open(filename.c_str());
    if( _pp_map_info ) {
        for( int i=0; i<_sampling_width; i++ ) {
            for( int j=0; j<_sampling_height; j++ ) {
                mapInfoFile << _pp_map_info[i][j] << " ";
            }
            mapInfoFile << std::endl;
        }
    }
    mapInfoFile.close();
}

bool MORRF::are_reference_structures_correct() {
    for( std::vector<ReferenceTree*>::iterator it=_references.begin(); it!=_references.end(); it++ ) {
        ReferenceTree* p_ref_tree = (*it);
        if( p_ref_tree ) {
            if( false == p_ref_tree->is_structure_correct() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::are_subproblem_structures_correct() {
    for( std::vector<SubproblemTree*>::iterator it=_subproblems.begin(); it!=_subproblems.end(); it++ ) {
        SubproblemTree* p_sub_tree = (*it);
        if( p_sub_tree ) {
            if( false == p_sub_tree->is_structure_correct() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::are_all_reference_nodes_tractable() {
    for( std::vector<ReferenceTree*>::iterator it=_references.begin(); it!=_references.end(); it++ ) {
        ReferenceTree* p_ref_tree = (*it);
        if( p_ref_tree ) {
            if( false == p_ref_tree->are_all_nodes_tractable() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::are_all_subproblem_nodes_tractable() {
    for( std::vector<SubproblemTree*>::iterator it=_subproblems.begin(); it!=_subproblems.end(); it++ ) {
        SubproblemTree* p_sub_tree = (*it);
        if( p_sub_tree ) {
            if( false == p_sub_tree->are_all_nodes_tractable() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::are_all_reference_nodes_fitness_positive() {
    for( std::vector<ReferenceTree*>::iterator it=_references.begin(); it!=_references.end(); it++ ) {
        ReferenceTree* p_ref_tree = (*it);
        if( p_ref_tree ) {
            if( false == p_ref_tree->are_all_nodes_fitness_positive() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::are_all_subproblem_nodes_fitness_positive() {
    for( std::vector<SubproblemTree*>::iterator it=_subproblems.begin(); it!=_subproblems.end(); it++ ) {
        SubproblemTree* p_sub_tree = (*it);
        if( p_sub_tree ) {
            if( false == p_sub_tree->are_all_nodes_fitness_positive() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::is_node_number_identical() {
    int ref_num = _references[0]->m_nodes.size();

    for(std::vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree) {
            int num = pRefTree->m_nodes.size();
            if(num != ref_num) {
                return false;
            }
        }
    }
    for(std::vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* pSubTree = (*it);
        if(pSubTree) {
            int num = pSubTree->m_nodes.size();
            if(num != ref_num) {
                return false;
            }
        }
    }
    return true;
}

std::vector<Path*> MORRF::get_paths() {
    std::vector<Path*> paths;

    for(std::vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree) {
            Path* pRefPath = pRefTree->find_path();
            paths.push_back(pRefPath);
        }
    }
    for(std::vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* pSubTree = (*it);
        if(pSubTree) {
            Path* pSubPath = pSubTree->find_path();
            paths.push_back(pSubPath);
        }
    }
    return paths;
}

bool MORRF::update_path_cost( Path *p ) {
    if(p)
    {
        for(int k=0;k<_objective_num;k++)
        {
            p->mp_cost[k] = 0.0;
        }
        for(int i=0;i<p->m_waypoints.size()-1;i++)
        {
            POS2D pos_a = p->m_waypoints[i];
            POS2D pos_b = p->m_waypoints[i+1];
            double deltaCost[_objective_num];
            calc_cost(pos_a, pos_b, deltaCost);

            for(int k=0;k<_objective_num;k++)
            {
                p->mp_cost[k] += deltaCost[k];
            }
        }        
        return true;
    }
    return false;
}

bool MORRF::is_ref_tree_min_cost() {
    if(_p_kd_tree) {
        for(KDTree2D::const_iterator it = _p_kd_tree->begin(); it!= _p_kd_tree->end(); it++) {
            KDNode2D node = (*it);
            double minCost[_objective_num];
            for(int k=0;k<_objective_num;k++) {
                minCost[k] = std::numeric_limits<double>::max();
            }
            for(int i=_objective_num; i<node.m_node_list.size(); i++) {
                RRTNode* pNode = node.m_node_list[i];
                for(int k=0;k<_objective_num;k++) {
                    if(pNode->mp_cost[k] < minCost[k]) {
                        minCost[k] = pNode->mp_cost[k];
                    }
                }
            }
            for(int k=0;k<_objective_num;k++) {
                RRTNode* pRefNode = node.m_node_list[k];
                if(pRefNode->m_fitness > minCost[k]) {
                    return false;
                }
            }
        }
    }
    return true;
}
