#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>
#include "morrf.h"

#define OBSTACLE_THRESHOLD 200

using namespace std;

MORRF::MORRF(unsigned int width, unsigned int height, unsigned int objective_num, unsigned int subproblem_num, unsigned int segmentLength, MORRF_TYPE type) {
    _sampling_width = width;
    _sampling_height = height;
    _objective_num = objective_num;
    _subproblem_num = subproblem_num;
    _type = type;

#ifndef USE_FLANN
    _p_kd_tree = new KDTree2D( std::ptr_fun(tac) );
#else
    _p_kd_tree = new KDTree2D();
#endif

    _range = ( _sampling_width > _sampling_height ) ? _sampling_width:_sampling_height;
    _ball_radius = _range;
    _obs_check_resolution = 1;
    _current_iteration = 0;
    _segment_length = segmentLength;

    _theta = 4;

    _pp_map_info = new int*[_sampling_width];
    for( unsigned int i=0; i<_sampling_width; i++ ) {
        _pp_map_info[i] = new int[_sampling_height];
        for( unsigned int j=0; j<_sampling_height; j++) {
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

    _weights.clear();

    for( unsigned int i=0; i<_subproblem_num; i++ ) {
        vector<double> weight( _objective_num, 0.0 );
        std::vector<float> temp_array;
        temp_array.push_back(0.0);
        for( unsigned int j=0; j<_objective_num-1; j++ ) {
            temp_array.push_back( static_cast<float>(rand())/static_cast<float>(RAND_MAX) );
        }
        temp_array.push_back(1.0);
        sort(temp_array.begin(), temp_array.end());
        for( unsigned int j=0; j<_objective_num; j++ ) {
            weight[j] = temp_array[j+1] - temp_array[j];
        } 
        _weights.push_back( weight );
    }
}

void MORRF::_deinit_weights() {
    _weights.clear();
}

void MORRF::init(POS2D start, POS2D goal) {

    _init_weights();

    KDNode2D root(start);
    for( unsigned int k=0; k<_objective_num; k++ ) {
        vector<double> weight(_objective_num, 0.0);
        weight[k] = 1.0;
        ReferenceTree * p_ref_tree = new ReferenceTree( this, _objective_num, weight, k );
        RRTNode * p_root_node = p_ref_tree->init( start, goal );
        root.m_node_list.push_back( p_root_node );
        _references.push_back( p_ref_tree );
    }

    for( unsigned int m=0; m<_subproblem_num; m++ ) {
        SubproblemTree * p_sub_tree = new SubproblemTree( this, _objective_num, _weights[m], m );
        RRTNode * p_root_node = p_sub_tree->init( start, goal );
        root.m_node_list.push_back( p_root_node );
        _subproblems.push_back( p_sub_tree );
    }
    _p_kd_tree->insert( root );
    _current_iteration = 0;
}

void MORRF::load_map(int **map) {
    for( unsigned int i=0; i<_sampling_width; i++ ) {
        for( unsigned int j=0; j<_sampling_height; j++ ) {
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
        POS2D rnd_pos = sampling();
        KDNode2D nearest_node = find_nearest( rnd_pos );

        POS2D new_pos = steer( rnd_pos, nearest_node );

        if(true == _contains( new_pos )) {
            continue;
        }
        if( true == _is_in_obstacle( new_pos ) ) {
            continue;
        }

        if( true == _is_obstacle_free( nearest_node, new_pos )) {
            std::list<KDNode2D> near_nodes = find_near( new_pos );
            KDNode2D new_node( new_pos );

            MORRFNode* p_morrf_node = new MORRFNode( new_pos );

            // create new nodes of reference trees
            for( unsigned int k=0; k < _objective_num; k++ ) {
                RRTNode * p_new_ref_node = _references[k]->create_new_node( new_pos );
                p_new_ref_node->mp_host_node = p_morrf_node;
                p_morrf_node->m_nodes.push_back( p_new_ref_node );
                new_node.m_node_list.push_back( p_new_ref_node );
            }

            // create new nodes of subproblem trees
            for ( unsigned int m=0; m < _subproblem_num; m++ ) {
                RRTNode * p_new_sub_node = _subproblems[m]->create_new_node( new_pos );
                p_new_sub_node->mp_host_node = p_morrf_node;
                p_morrf_node->m_nodes.push_back( p_new_sub_node );
                new_node.m_node_list.push_back( p_new_sub_node );
            }

            _morrf_nodes.push_back( p_morrf_node );

            _p_kd_tree->insert( new_node );
            node_inserted = true;

            // attach new node to reference trees
            // rewire near nodes of reference trees
            for ( unsigned int k=0; k<_objective_num; k++ ) {
                // std::cout << "@ " << k << std::endl;
                unsigned int index = k;
                RRTNode* p_nearest_ref_node = nearest_node.m_node_list[index];
                RRTNode* p_new_ref_node = new_node.m_node_list[index];
                list<RRTNode*> near_ref_nodes;
                near_ref_nodes.clear();
                for( list<KDNode2D>::iterator itr = near_nodes.begin();
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
            for( unsigned int m=0; m<_subproblem_num; m++ ) {
                // std::cout << "@ " << m+mObjectiveNum << std::endl;
                unsigned int index = m + _objective_num;
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

    if(_current_iteration % 10 == 0) {
        optimize();
    }

    _current_iteration++;
}

KDNode2D MORRF::find_nearest( POS2D pos ) {
    KDNode2D node( pos );
#ifndef USE_FLANN
    std::pair<KDTree2D::const_iterator,double> found = _p_kd_tree->find_nearest( node );
    KDNode2D nearest_node = *found.first;
    return nearest_node;
#else
    KDNode2D *p_result = _p_kd_tree->find_nearest( node );
    if( p_result ) {
        return *p_result;
    }
    return node;
#endif
}

std::list<KDNode2D> MORRF::find_near( POS2D pos ) {
    std::list<KDNode2D> near_list;
    KDNode2D node(pos);

    int num_vertices = _p_kd_tree->size();
    int num_dimensions = 2;
    _ball_radius = _theta * _range * pow( log((double)(num_vertices + 1.0))/((double)(num_vertices + 1.0)), 1.0/((double)num_dimensions) );
#ifndef USE_FLANN
    _p_kd_tree->find_within_range( node, _ball_radius, std::back_inserter(near_list) );
#else
    _p_kd_tree->find_within_range( node, _ball_radius, near_list );
#endif

    return near_list;
}


bool MORRF::_contains( POS2D pos ) {
    if( _p_kd_tree ) {
        KDNode2D node( pos[0], pos[1] );
#ifndef USE_FLANN
        KDTree2D::const_iterator it = _p_kd_tree->find( node );
        if( it != _p_kd_tree->end() ) {
            return true;
        }
        else {
            return false;
        }
#else
        if( _p_kd_tree->find( node ) ) {
            return true;
        }
#endif
    }
    return false;
}

bool MORRF::calc_cost(POS2D& pos_a, POS2D& pos_b, std::vector<double>& cost) {
    for( unsigned int k = 0; k < _objective_num; k++ ) {
        cost[k] = calc_kth_cost( pos_a, pos_b, k );
    }
    return true;
}

double MORRF::calc_kth_cost( POS2D& pos_a, POS2D& pos_b, unsigned int k ) {
    return _funcs[k]( pos_a, pos_b, _fitness_distributions[k], (void*)this );
}

double MORRF::calc_fitness( vector<double>& cost, vector<double>& weight, RRTNode* node ) {
    double fitness = 0.0;

    if( _type==MORRF::WEIGHTED_SUM ) {
        fitness = calc_fitness_by_weighted_sum( cost, weight );
    }
    else if( _type==MORRF::TCHEBYCHEFF ) {
        vector<double> utopia( _objective_num, 0.0 );
        get_utopia_reference_vector( node, utopia );
        fitness = calc_fitness_by_tchebycheff( cost, weight, utopia );
    }
    else {
        vector<double> utopia( _objective_num, 0.0 );
        get_utopia_reference_vector( node, utopia );
        fitness = calc_fitness_by_boundary_intersection( cost, weight, utopia );
    }
    /*
    if(fitness < 0.0) {
        std::cout << "Negative fitness " << fitness << std::endl;
    } */
    return fitness;
}

double MORRF::calc_fitness( vector<double>& cost, vector<double>& weight, POS2D& pos ) {
    double fitness = 0.0;

    if( _type==MORRF::WEIGHTED_SUM ) {
        fitness = calc_fitness_by_weighted_sum( cost, weight );
    }
    else if( _type==MORRF::TCHEBYCHEFF ) {
        vector<double> utopia(_objective_num, 0.0);
        get_utopia_reference_vector( pos, utopia );
        fitness = calc_fitness_by_tchebycheff( cost, weight, utopia );
    }
    else {
         vector<double> utopia(_objective_num, 0.0);
        get_utopia_reference_vector( pos, utopia );
        fitness = calc_fitness_by_boundary_intersection( cost, weight, utopia );
    }
    /*
    if(fitness < 0.0) {
        std::cout << "Negative fitness " << fitness << std::endl;
    } */
    return fitness;
}


float MORRF::calc_fitness_by_weighted_sum( vector<double>& cost, vector<double>& weight ) {
    double fitness = 0.0;
    for( unsigned int k=0; k<_objective_num; k++ ) {
        fitness += cost[k] * weight[k];
    }
    return fitness;
}

float MORRF::calc_fitness_by_tchebycheff( vector<double>& cost, vector<double>& weight, vector<double>& utopia_reference ) {
    std::vector<float> weighted_distance(_objective_num, 0.0);
    for( unsigned int k=0; k<_objective_num; k++ ) {
       weighted_distance[k] = weight[k] * fabs( cost[k] - utopia_reference[k] );
    }
    sort(weighted_distance.begin(), weighted_distance.end());
    return weighted_distance.back();
}

float MORRF::calc_fitness_by_boundary_intersection( vector<double>& cost, vector<double>& weight, vector<double>& utopia_reference ) {
    double d1 = 0.0, d2 = 0.0;
    for( unsigned int k=0; k<_objective_num; k++ ) {
       double weighted_dist = weight[k] * (cost[k] - utopia_reference[k]);
       d1 += weighted_dist;
    }
    d1 = fabs(d1);
    vector<double> vectorD2(_objective_num, 0.0);
    for( unsigned int k=0; k<_objective_num; k++ ) {
        vectorD2[k] = cost[k] - (utopia_reference[k] + d1* weight[k]);
        d2 += vectorD2[k]*vectorD2[k];
    }
    d2 = sqrt(d2);
    return d1 + _theta * d2;
}


bool MORRF::get_utopia_reference_vector(POS2D&  pos, vector<double>& utopia ) {

    KDNode2D ref_node = find_nearest(pos);
    if( ref_node.m_node_list.size()<_objective_num ) {
        return false;
    }

    for( unsigned int k=0; k<_objective_num; k++ ) {
        RRTNode* p_RRT_node = ref_node.m_node_list[k];
        utopia[k] = p_RRT_node->m_fitness;
    }
    return true;
}

bool MORRF::get_utopia_reference_vector( RRTNode* p_node, vector<double>& utopia ) {
    if( p_node == NULL ) {
         return false;
    }
    if( p_node && p_node->mp_host_node ) {
        for( unsigned int k=0; k<_objective_num; k++ ) {
            utopia[k] = p_node->mp_host_node->m_nodes[k]->m_fitness;
        }
    }
    return true;
}

ReferenceTree* MORRF::get_reference_tree(unsigned int k) {
    if( k<0 || k>=_objective_num ) {
        return NULL;
    }
    return _references[k];
}

SubproblemTree* MORRF::get_subproblem_tree( unsigned int m ) {
    if( m<0 || m>=_subproblem_num ) {
        return NULL;
    }
    return _subproblems[m];
}

void MORRF::optimize() {
#ifndef USE_FLANN
    if(_p_kd_tree) {
        _p_kd_tree->optimize();
    }
#endif
}

void MORRF::dump_map_info( std::string filename ) {
    ofstream map_info_file;
    map_info_file.open(filename.c_str());
    if( _pp_map_info ) {
        for( unsigned int i=0; i<_sampling_width; i++ ) {
            for( unsigned int j=0; j<_sampling_height; j++ ) {
                map_info_file << _pp_map_info[i][j] << " ";
            }
            map_info_file << std::endl;
        }
    }
    map_info_file.close();
}

void MORRF::dump_weights( std::string filename ) {
    ofstream weight_file;
    weight_file.open(filename.c_str());

    for( unsigned int i=0; i<_subproblem_num; i++ ) {
        for( unsigned int j=0; j<_objective_num; j++ ) {
            weight_file << _weights[i][j] << " ";
        }
        weight_file << std::endl;
    }

    weight_file.close();
}

bool MORRF::are_reference_structures_correct() {
    for( vector<ReferenceTree*>::iterator it=_references.begin(); it!=_references.end(); it++ ) {
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
    for( vector<SubproblemTree*>::iterator it=_subproblems.begin(); it!=_subproblems.end(); it++ ) {
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
    for( vector<ReferenceTree*>::iterator it=_references.begin(); it!=_references.end(); it++ ) {
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
    for( vector<SubproblemTree*>::iterator it=_subproblems.begin(); it!=_subproblems.end(); it++ ) {
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
    for( vector<SubproblemTree*>::iterator it=_subproblems.begin(); it!=_subproblems.end(); it++ ) {
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
    unsigned int ref_num = _references[0]->m_nodes.size();

    for(vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* p_ref_tree = (*it);
        if(p_ref_tree) {
            unsigned int num = p_ref_tree->m_nodes.size();
            if(num != ref_num) {
                return false;
            }
        }
    }
    for(vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* p_sub_tree = (*it);
        if(p_sub_tree) {
            unsigned int num = p_sub_tree->m_nodes.size();
            if(num != ref_num) {
                return false;
            }
        }
    }
    return true;
}

vector<Path*> MORRF::get_paths() {
    vector<Path*> paths;

    for(vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* p_ref_tree = (*it);
        if(p_ref_tree) {
            Path* pRefPath = p_ref_tree->find_path();
            paths.push_back(pRefPath);
        }
    }
    for(vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* p_sub_tree = (*it);
        if(p_sub_tree) {
            Path* pSubPath = p_sub_tree->find_path();
            paths.push_back(pSubPath);
        }
    }
    return paths;
}

bool MORRF::update_path_cost( Path *p ) {
    if(p) {
        for(unsigned int k=0;k<_objective_num;k++) {
            p->m_cost[k] = 0.0;
        }
        for(unsigned int i=0;i<p->m_waypoints.size()-1;i++) {
            POS2D pos_a = p->m_waypoints[i];
            POS2D pos_b = p->m_waypoints[i+1];
            vector<double> delta_cost(_objective_num, 0.0);
            calc_cost(pos_a, pos_b, delta_cost);

            for(unsigned int k=0;k<_objective_num;k++) {
                p->m_cost[k] += delta_cost[k];
            }
        }        
        return true;
    }
    return false;
}

bool MORRF::is_ref_tree_min_cost() {
    if(_p_kd_tree) {
#ifndef USE_FLANN
        for(KDTree2D::const_iterator it = _p_kd_tree->begin(); it!= _p_kd_tree->end(); it++) {
            KDNode2D node = (*it);
            vector<double> min_cost(_objective_num, std::numeric_limits<double>::max());

            for(unsigned int i=_objective_num; i<node.m_node_list.size(); i++) {
                RRTNode* p_node = node.m_node_list[i];
                for(unsigned int k=0;k<_objective_num;k++) {
                    if(p_node->m_cost[k] < min_cost[k]) {
                        min_cost[k] = p_node->m_cost[k];
                    }
                }
            }
            for(unsigned int k=0;k<_objective_num;k++) {
                RRTNode* p_ref_node = node.m_node_list[k];
                if(p_ref_node->m_fitness > min_cost[k]) {
                    return false;
                }
            }
        }
#endif
    }
    return true;
}
