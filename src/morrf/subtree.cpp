#include <limits>
#include "morrf/morrf.h"
#include "morrf/subtree.h"

using namespace std;

RRTNode::RRTNode( POS2D pos, int objective_num ) {
    m_pos = pos;
    m_objective_num = objective_num;

    m_cost = vector<double>(m_objective_num, 0.0);
    m_fitness = 0.0;
    mp_parent = NULL;
    mp_host_node = NULL;
}

bool RRTNode::operator==( const RRTNode &other ) {
    return m_pos==other.m_pos;
}

Path::Path( POS2D start, POS2D goal, int objectiveNum ) {
    m_start = start;
    m_goal = goal;
    m_objective_num = objectiveNum;
    m_cost = vector<double>(m_objective_num, 0.0);
    m_weight = vector<double>(m_objective_num, 0.0);
    m_fitness = 0.0;
}

RRTree::RRTree( MORRF* parent, unsigned int objective_num, std::vector<float>  weight, unsigned int index ) {
    mp_parent = parent;
    m_type = UNKNOWN;
    m_objective_num = objective_num;
    m_index = index;
    mp_current_best = NULL;

    m_weight.clear();
    for( unsigned int k=0; k<m_objective_num; k++ ) {
        m_weight.push_back( weight[k] );
    }
    mp_root = NULL;

    m_nodes.clear();
    m_current_best_cost = std::vector<double>(m_objective_num, 0.0);
    m_current_best_fitness = std::numeric_limits<float>::max();
    m_first_path_iteration = 0;
    m_sparsity_level = std::numeric_limits<float>::max();
}

RRTNode* RRTree::init( POS2D start, POS2D goal ) {
    if(mp_root) {
        delete mp_root;
        mp_root = NULL;
    }
    m_start = start;
    m_goal = goal;
    mp_root = new RRTNode( start, m_objective_num );
    m_nodes.push_back(mp_root);

    return mp_root;
}

RRTNode*  RRTree::create_new_node( POS2D pos ) {
    RRTNode * pNode = new RRTNode( pos, m_objective_num );
    m_nodes.push_back(pNode);
    return pNode;
}

bool RRTree::remove_edge( RRTNode* p_node_p, RRTNode*  p_node_c ) {
    if(p_node_p==NULL) {
        return false;
    }

    p_node_c->mp_parent = NULL;
    bool removed = false;
    for( std::list<RRTNode*>::iterator it = p_node_p->m_child_nodes.begin(); it != p_node_p->m_child_nodes.end(); it++ ) {
        RRTNode* pCurrent = (RRTNode*)(*it);
        if ( pCurrent==p_node_c || pCurrent->m_pos==p_node_c->m_pos ) {
            pCurrent->mp_parent = NULL;
            it = p_node_p->m_child_nodes.erase(it);
            removed = true;
        }
    }
    return removed;
}

bool RRTree::has_edge( RRTNode* p_node_p, RRTNode* p_node_c ) {
    if ( p_node_p==NULL || p_node_c==NULL ) {
        return false;
    }
    for( std::list<RRTNode*>::iterator it=p_node_p->m_child_nodes.begin(); it!=p_node_p->m_child_nodes.end(); it++ ) {
        RRTNode* p_curr_node = (*it);
        if( p_curr_node==p_node_c ) {
            return true;
        }
    }
    /*
    if ( p_node_p == p_node_c->mp_parent)
        return true;
    */
    return false;
}

bool RRTree::add_edge( RRTNode* p_node_p, RRTNode* p_node_c ) {
    if( p_node_p==NULL || p_node_c==NULL || p_node_p==p_node_c ) {
        return false;
    }
    if ( p_node_p->m_pos == p_node_c->m_pos ) {
        return false;
    }
    if ( true == has_edge( p_node_p, p_node_c ) ) {
        p_node_c->mp_parent = p_node_p;
    }
    else {
        p_node_p->m_child_nodes.push_back( p_node_c );
        p_node_c->mp_parent = p_node_p;
    }
    p_node_c->m_child_nodes.unique();

    return true;
}


list<RRTNode*> RRTree::find_all_children( RRTNode* p_node ) {
    int level = 0;
    bool finished = false;
    list<RRTNode*> child_list;

    list<RRTNode*> current_level_nodes;
    current_level_nodes.push_back( p_node );
    while( false==finished ) {
        list<RRTNode*> current_level_children;
        unsigned int child_list_num = child_list.size();

        for( list<RRTNode*>::iterator it=current_level_nodes.begin(); it!=current_level_nodes.end(); it++ ) {
            RRTNode* p_current_node = (*it);
            for( list<RRTNode*>::iterator itc = p_current_node->m_child_nodes.begin(); itc != p_current_node->m_child_nodes.end(); itc++ ) {
                RRTNode *p_child_node = (*itc);
                if( p_child_node ) {
                    current_level_children.push_back( p_child_node );
                    child_list.push_back( p_child_node );
                }
            }
        }

        child_list.unique();
        current_level_children.unique();

        if ( current_level_children.size() == 0 ) {
            finished = true;
        }
        else if ( child_list.size() == child_list_num ) {
            finished = true;
        }
        else {
            current_level_nodes.clear();
            for( list<RRTNode*>::iterator itt = current_level_children.begin(); itt != current_level_children.end(); itt++ ) {
                RRTNode * p_temp_node = (*itt);
                if( p_temp_node ) {
                    current_level_nodes.push_back( p_temp_node );
                }
            }
            level += 1;
        }

        if( level > 100 ) {
            break;
        }
    }
    child_list.unique();
    return child_list;
}

bool RRTree::is_structure_correct() {
    for( list<RRTNode*>::iterator it = m_nodes.begin(); it != m_nodes.end(); it++ ) {
        RRTNode * p_node = (*it);
        if( p_node ) {
            for( list<RRTNode*>::iterator itc=p_node->m_child_nodes.begin(); itc!=p_node->m_child_nodes.end(); itc++ ) {
                RRTNode * p_child_node = (*itc);
                if( p_child_node ) {
                    if( p_child_node->m_fitness < p_node->m_fitness ) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

RRTNode* RRTree::find_ancestor( RRTNode* p_node ) {
    return get_ancestor( p_node );
}

Path* RRTree::find_path(RRTNode * p_closest_node) {
    Path* p_new_path = new Path( m_start, m_goal, m_objective_num );

    list<RRTNode*> node_list;

    if(p_closest_node==NULL) {
        vector<double> delta_cost = vector<double>(m_objective_num, 0.0);
        double delta_fitness = 0.0;
        p_closest_node = get_closet_to_goal( delta_cost, delta_fitness );
    }

    if( p_closest_node!=NULL ) {

        if( mp_parent ) {
            if( mp_parent->_is_obstacle_free(m_goal, p_closest_node->m_pos) == false ) {
                return NULL;
            }
            get_parent_node_list( p_closest_node, node_list );

            for( list<RRTNode*>::reverse_iterator rit=node_list.rbegin();
                rit!=node_list.rend(); ++rit ) {
                RRTNode* pNode = (*rit);
                p_new_path->m_waypoints.push_back( pNode->m_pos );
            }
            p_new_path->m_waypoints.push_back( m_goal );

            /*
            for( unsigned int k=0; k<m_objective_num; k++ ) {
                p_new_path->m_cost[k] = p_first_node->m_cost[k] + delta_cost[k];
                p_new_path->m_weight[k] = m_weight[k];
            }
            p_new_path->m_fitness = p_first_node->m_fitness + delta_fitness;
            */
            mp_parent->update_path_cost( p_new_path );
        }
    }
    else {
        return NULL;
    }

    return p_new_path;
}

bool RRTree::are_all_nodes_tractable() {
    for( std::list<RRTNode*>::iterator it=m_nodes.begin(); it!=m_nodes.end(); it++ ) {
        RRTNode * p_node = (*it);
        if( p_node ) {
            if( mp_root != find_ancestor( p_node ) ) {
                return false;
            }
        }
    }
    return true;
}

bool RRTree::are_all_nodes_fitness_positive() {
    for( std::list<RRTNode*>::iterator it=m_nodes.begin(); it!=m_nodes.end(); it++ ) {
        RRTNode * p_node = (*it);
        if( p_node ) {
            if( p_node->m_fitness < 0.0 ) {
                return false;
            }
        }
    }
    return true;
}

bool RRTree::update_current_best(RRTNode* p_closet_node) {

    mp_current_best = find_path(p_closet_node);
    if( mp_current_best ) {
        for(unsigned int k=0;k<m_objective_num;k++) {
            m_current_best_cost[k] = mp_current_best->m_cost[k];
        }
        return true;
    }
    return false;
}

void RRTree::record() {
    if(mp_current_best==NULL) {
        m_first_path_iteration++;
    }
    else {
        m_hist_cost.push_back( m_current_best_cost );
        m_hist_fitness.push_back( m_current_best_fitness );
        m_hist_sparsity_level.push_back( m_sparsity_level );
    }
}

void RRTree::write_hist_data( std::ostream& out ) {

    for(unsigned int k=0;k<m_objective_num;k++) {
        for(unsigned int i=0;i<m_hist_cost.size();i++) {
            out << m_hist_cost[i][k] << " ";
        }
        out << std::endl;
    }
    for(unsigned int j=0;j<m_hist_fitness.size();j++) {
        out << m_hist_fitness[j] << " ";
    }
    out << std::endl;
    for(unsigned int j=0;j<m_hist_sparsity_level.size();j++) {
        out << m_hist_sparsity_level[j] << " ";
    }
    out << std::endl;
    for(unsigned int j=m_first_path_iteration;
        j<m_first_path_iteration+m_hist_fitness.size();j++) {
        out << j << " ";
    }
    out << std::endl;

}


ReferenceTree::ReferenceTree( MORRF* parent, unsigned int objective_num, std::vector<float> weight, unsigned int index )
    : RRTree( parent, objective_num, weight, index ) {
    m_type = REFERENCE;
}

void ReferenceTree::attach_new_node( RRTNode* p_node_new, std::list<RRTNode*> near_nodes ) {
    double min_new_node_fitness = std::numeric_limits<double>::max();
    RRTNode* p_min_node = NULL;

    for( std::list<RRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++ ) {
        RRTNode* p_near_node = *it;
        if( p_near_node->m_pos == p_node_new->m_pos ) {
            continue;
        }

        if ( true == mp_parent->_is_obstacle_free( p_near_node->m_pos, p_node_new->m_pos ) ) {
            double fitness = p_near_node->m_fitness + mp_parent->calc_kth_cost( p_near_node->m_pos, p_node_new->m_pos, m_index );
            if (fitness < min_new_node_fitness || p_min_node == NULL ) {
                p_min_node = p_near_node;
                min_new_node_fitness = fitness;
            }
        }
    }

    bool added = add_edge(p_min_node, p_node_new);
    if(added) {
        p_node_new->m_fitness = min_new_node_fitness;
        p_node_new->m_cost[m_index] = p_node_new->m_fitness;
    }
}

void ReferenceTree::rewire_near_nodes( RRTNode* p_node_new, std::list<RRTNode*> near_nodes ) {
    for( std::list<RRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++ ) {
        RRTNode * p_near_node = (*it);

        if( p_near_node->m_pos ==p_node_new->m_pos ||  p_near_node->m_pos==mp_root->m_pos || p_node_new->mp_parent->m_pos==p_near_node->m_pos ) {
            continue;
        }

        if( true == mp_parent->_is_obstacle_free( p_node_new->m_pos, p_near_node->m_pos ) ) {
            double temp_fitness_from_new_node = p_node_new->m_fitness + mp_parent->calc_kth_cost( p_node_new->m_pos, p_near_node->m_pos, m_index );
            if(temp_fitness_from_new_node < p_near_node->m_fitness) {
                double delta_fitness = p_near_node->m_fitness - temp_fitness_from_new_node;
                RRTNode * p_parent_node = p_near_node->mp_parent;
                bool removed = remove_edge( p_parent_node, p_near_node );
                if( removed ) {
                    bool added = add_edge( p_node_new, p_near_node );
                    if( added ) {
                        p_near_node->m_fitness = temp_fitness_from_new_node;
                        p_near_node->m_cost[m_index] = p_near_node->m_fitness;
                        update_fitness_to_children(p_near_node, delta_fitness);
                    }
                }
                else {
                    std::cout << " Failed in removing " << std::endl;
                }
            }
        }
    }
}

void ReferenceTree::update_fitness_to_children( RRTNode* p_node, double delta_fitness ) {
    list<RRTNode*> child_list = find_all_children(p_node);
    for( list<RRTNode*>::iterator it=child_list.begin(); it!=child_list.end(); it++ ) {
        RRTNode* p_child_node = (*it);
        if( p_child_node ) {
            p_child_node->m_fitness -= delta_fitness;
            p_child_node->m_cost[m_index] = p_child_node->m_fitness;
        }
    }
}

RRTNode * ReferenceTree::get_closet_to_goal( vector<double>& delta_cost, double& delta_fitness ) {
    RRTNode* p_closest_node = NULL;
    if( mp_parent ) {
        /*
        KDNode2D nearest_node = mp_parent->find_nearest( m_goal );
        p_closest_node = nearest_node.mp_morrf_node->m_nodes[m_index];
        */
        list<KDNode2D> near_nodes = mp_parent->find_near( m_goal );
        double min_total_fitness = std::numeric_limits<double>::max();
        double min_delta_fitness = 0.0;
        RRTNode * p_min_prev_node = NULL;
        for( list<KDNode2D>::iterator it=near_nodes.begin();
            it!=near_nodes.end(); it++ ) {

            KDNode2D kd_node = (*it);
            RRTNode* p_node = kd_node.mp_morrf_node->m_nodes[m_index];
            if( mp_parent->_is_obstacle_free( p_node->m_pos, m_goal ) ) {

                double delta_fitness = mp_parent->calc_kth_cost( p_node->m_pos, m_goal, m_index );
                double new_total_fitness = p_node->m_fitness + delta_fitness;
                if ( new_total_fitness < min_total_fitness || p_min_prev_node == NULL ) {
                    p_min_prev_node = p_node;
                    min_delta_fitness = delta_fitness;
                    min_total_fitness = new_total_fitness;
                }
            }
        }
        p_closest_node = p_min_prev_node;
        delta_fitness = min_delta_fitness;
        for( unsigned int k=0; k<m_objective_num; k++ ) {
            if( k == m_index ) {
                delta_cost[k] = delta_fitness;
            }
            else {
                delta_cost[k] = 0.0;
            }
        }
    }
    return p_closest_node;
}

Path* ReferenceTree::find_path(RRTNode* p_closet_node) {
    Path* p_new_path = RRTree::find_path(p_closet_node);
    if( p_new_path ) {
        if( mp_parent ) {
            mp_parent->update_path_cost( p_new_path );
        }
    }
    return p_new_path;
}

bool ReferenceTree::update_current_best(RRTNode* p_closet_node) {
    mp_current_best = find_path(p_closet_node);
    if( mp_current_best ) {
        for(unsigned int k=0;k<m_objective_num;k++) {
            m_current_best_cost[k] = mp_current_best->m_cost[k];
        }
        return true;
    }
    return false;
}

SubproblemTree::SubproblemTree( MORRF* parent, unsigned int objective_num, vector<float> weight, unsigned int index )
    : RRTree(parent, objective_num, weight, index ) {
    m_type = SUBPROBLEM;
}

void SubproblemTree::attach_new_node( RRTNode* p_node_new, list<RRTNode*> near_nodes ) {
    vector<double> min_new_node_cost( m_objective_num, 0.0 );

    RRTNode* p_min_node = NULL;
    double min_new_node_fitness = std::numeric_limits<double>::max();


    for( list<RRTNode*>::iterator it = near_nodes.begin(); it != near_nodes.end(); it++ ) {
        RRTNode* p_near_node = (*it);

        if(p_near_node->m_pos == p_node_new->m_pos){
            continue;
        }
        if ( true == mp_parent->_is_obstacle_free(p_near_node->m_pos, p_node_new->m_pos) ) {
            vector<double> cost_temp(m_objective_num, 0.0);
            vector<double> cost_delta(m_objective_num, 0.0);
            mp_parent->calc_cost(p_near_node->m_pos, p_node_new->m_pos, cost_delta);
            for( unsigned int k=0; k < m_objective_num; k++ ) {
                cost_temp[k] = p_near_node->m_cost[k] + cost_delta[k];
            }
            double fitness = mp_parent->calc_fitness(cost_temp, m_weight, p_node_new);
            if ( fitness < min_new_node_fitness || p_min_node == NULL ) {
                p_min_node = p_near_node;
                min_new_node_fitness = fitness;
                for( unsigned int k = 0; k < m_objective_num; k++ ) {
                    min_new_node_cost[k] = cost_temp[k];
                }
            }
        }
    }

    bool added = add_edge( p_min_node, p_node_new );
    if( added ) {
        p_node_new->m_fitness = min_new_node_fitness;
        for( unsigned int k = 0; k < m_objective_num; k++ ) {
           p_node_new->m_cost[k] = min_new_node_cost[k];
        }
    }
}

void SubproblemTree::rewire_near_nodes( RRTNode* p_node_new, list<RRTNode*> near_nodes ) {
    for( list<RRTNode*>::iterator it = near_nodes.begin(); it != near_nodes.end(); it++ ) {
        RRTNode * p_near_node = (*it);

        if( p_near_node->m_pos == p_node_new->m_pos ||  p_near_node->m_pos == mp_root->m_pos || p_node_new->mp_parent->m_pos == p_near_node->m_pos ) {
            continue;
        }

        if( true == mp_parent->_is_obstacle_free( p_node_new->m_pos, p_near_node->m_pos ) ) {

            vector<double> temp_cost_from_new_node(m_objective_num, 0.0);
            vector<double> temp_delta_cost_from_new_node(m_objective_num, 0.0);

            mp_parent->calc_cost( p_node_new->m_pos, p_near_node->m_pos, temp_delta_cost_from_new_node );
            for( unsigned int k=0; k<m_objective_num; k++ ) {
                temp_cost_from_new_node[k] = p_node_new->m_cost[k] + temp_delta_cost_from_new_node[k];
            }
            double temp_fitness_from_new_node = mp_parent->calc_fitness( temp_cost_from_new_node, m_weight, p_near_node );

            if( temp_fitness_from_new_node < p_near_node->m_fitness ) {
                RRTNode * p_parent_node = p_near_node->mp_parent;
                bool removed = remove_edge( p_parent_node, p_near_node );
                if( removed ) {
                    bool added = add_edge( p_node_new, p_near_node );
                    if( added ) {
                        vector<double> delta_cost(m_objective_num, 0.0);
                        for( unsigned int k=0; k<m_objective_num; k++ ) {
                            delta_cost[k] = p_near_node->m_cost[k] - temp_cost_from_new_node[k];
                            p_near_node->m_cost[k] = temp_cost_from_new_node[k];
                        }
                        p_near_node->m_fitness = temp_fitness_from_new_node;
                        update_cost_to_children( p_near_node, delta_cost );
                    }
                }
            }
        }
    }
}

void SubproblemTree::update_cost_to_children( RRTNode* p_node, vector<double>& delta_cost ) {
    std::list<RRTNode*> child_list = find_all_children( p_node );
    for( std::list<RRTNode*>::iterator it=child_list.begin(); it!=child_list.end(); it++ ) {
        RRTNode* p_child_node = (*it);
        if( p_child_node ) {
            for( unsigned int k=0; k<m_objective_num; k++ ) {
                p_child_node->m_cost[k] -= delta_cost[k];
            }
            p_child_node->m_fitness = mp_parent->calc_fitness( p_child_node->m_cost, m_weight, p_child_node );
        }
    }
}

RRTNode * SubproblemTree::get_closet_to_goal( vector<double>& delta_cost, double& delta_fitness ){
    RRTNode * p_closest_node = NULL;
    /*
    if( mp_parent ) {
        KDNode2D nearest_node = mp_parent->find_nearest( m_goal );
        p_closest_node = nearest_node.mp_morrf_node->m_nodes[m_index];
    }
    */
    list<KDNode2D> near_nodes = mp_parent->find_near( m_goal );
    double min_total_fitness = std::numeric_limits<double>::max();
    double min_delta_fitness = 0.0;
    RRTNode * p_min_prev_node = NULL;
    for( list<KDNode2D>::iterator it=near_nodes.begin();
        it!=near_nodes.end(); it++ ) {

        KDNode2D kd_node = (*it);
        RRTNode* p_node = kd_node.mp_morrf_node->m_nodes[m_index];
        if( mp_parent->_is_obstacle_free( p_node->m_pos, m_goal ) ) {
            std::vector<double> delta_cost(m_objective_num, 0.0);
            std::vector<double> total_cost(m_objective_num, 0.0);
            mp_parent->calc_cost( p_node->m_pos, m_goal, delta_cost);
            for(unsigned int k=0;k<m_objective_num;k++) {
                total_cost[k] = p_node->m_cost[k] + delta_cost[k];
            }
            double new_total_fitness = mp_parent->calc_fitness( total_cost, m_weight, mp_parent->_solution_utopia );
            if ( new_total_fitness < min_total_fitness || p_min_prev_node == NULL ) {
                p_min_prev_node = p_node;
                min_delta_fitness = delta_fitness;
                min_total_fitness = new_total_fitness;
            }
        }
    }
    p_closest_node = p_min_prev_node;
    delta_fitness = min_delta_fitness;
    for( unsigned int k=0; k<m_objective_num; k++ ) {
        if( k == m_index ) {
            delta_cost[k] = delta_fitness;
        }
        else {
            delta_cost[k] = 0.0;
        }
    }

    return p_closest_node;
}
