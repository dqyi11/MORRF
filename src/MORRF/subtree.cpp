#include "subtree.h"
#include "morrf.h"
#include <limits>

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

RRTree::RRTree( MORRF* parent, unsigned int objective_num, std::vector<double>  weight, unsigned int index ) {
    mp_parent = parent;
    m_type = REFERENCE;
    m_objective_num = objective_num;
    m_index = index;

    m_weight.clear();
    for( unsigned int k=0; k<m_objective_num; k++ ) {
        m_weight.push_back( weight[k] );
    }

    mp_root = NULL;
    m_nodes.clear();
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

Path* RRTree::find_path() {
    Path* p_new_path = new Path( m_start, m_goal, m_objective_num );

    list<RRTNode*> node_list;

    RRTNode * p_first_node = NULL;
    vector<double> delta_cost = vector<double>(m_objective_num, 0.0);
    double delta_fitness = 0.0;
    p_first_node = get_closet_to_goal( delta_cost, delta_fitness );

    if( p_first_node!=NULL ) {
        get_parent_node_list( p_first_node, node_list );
        for( list<RRTNode*>::reverse_iterator rit=node_list.rbegin();
            rit!=node_list.rend(); ++rit ) {
            RRTNode* pNode = (*rit);
            p_new_path->m_waypoints.push_back( pNode->m_pos );
        }
        p_new_path->m_waypoints.push_back( m_goal );

        for( unsigned int k=0; k<m_objective_num; k++ ) {
            p_new_path->m_cost[k] = p_first_node->m_cost[k] + delta_cost[k];
            p_new_path->m_weight[k] = m_weight[k];
        }
        p_new_path->m_fitness = p_first_node->m_fitness + delta_fitness;
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


ReferenceTree::ReferenceTree( MORRF* parent, unsigned int objective_num, std::vector<double> weight, unsigned int index )
    : RRTree( parent, objective_num, weight, index ) {
    m_type = REFERENCE;
}


void ReferenceTree::attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes ) {
    double min_new_node_fitness = p_nearest_node->m_fitness + mp_parent->calc_kth_cost( p_nearest_node->m_pos, p_node_new->m_pos, m_index );
    RRTNode* p_min_node = p_nearest_node;

    for( std::list<RRTNode*>::iterator it=near_nodes.begin(); it!=near_nodes.end(); it++ ) {
        RRTNode* pNearNode = *it;
        if ( true == mp_parent->_is_obstacle_free( pNearNode->m_pos, p_node_new->m_pos ) ) {
            double fitness = pNearNode->m_fitness + mp_parent->calc_kth_cost( pNearNode->m_pos, p_node_new->m_pos, m_index );
            if (fitness < min_new_node_fitness) {
                p_min_node = pNearNode;
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
        list<KDNode2D> near_nodes = mp_parent->find_near( m_goal );
        double min_total_fitness = std::numeric_limits<double>::max();
        double min_delta_fitness = 0.0;
        RRTNode * p_min_prev_node = NULL;
        for( list<KDNode2D>::iterator it=near_nodes.begin();
            it!=near_nodes.end(); it++ ) {

            KDNode2D kd_node = (*it);
            RRTNode* p_node = kd_node.m_node_list[m_index];           
            if( mp_parent->_is_obstacle_free( p_node->m_pos, m_goal ) ) {

                double delta_fitness = mp_parent->calc_kth_cost( p_node->m_pos, m_goal, m_index );
                double new_total_fitness = p_node->m_fitness + delta_fitness;
                if ( new_total_fitness < min_total_fitness ) {
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

Path* ReferenceTree::find_path() {
    Path* p_new_path = RRTree::find_path();
    if( mp_parent ) {
        mp_parent->update_path_cost( p_new_path );
    }
    return p_new_path;
}


SubproblemTree::SubproblemTree( MORRF* parent, unsigned int objective_num, vector<double> weight, unsigned int index )
    : RRTree(parent, objective_num, weight, index ) {
    m_type = SUBPROBLEM;
}

void SubproblemTree::attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, list<RRTNode*> near_nodes ) {
    vector<double> min_new_node_cost( m_objective_num, 0.0 );
    vector<double> min_new_node_cost_delta( m_objective_num, 0.0 );
    mp_parent->calc_cost( p_nearest_node->m_pos, p_node_new->m_pos, min_new_node_cost_delta );
    for( unsigned int k = 0; k < m_objective_num; k++ ) {
        min_new_node_cost[k] = p_nearest_node->m_cost[k] + min_new_node_cost_delta[k];
    }
    double min_new_node_fitness = mp_parent->calc_fitness( min_new_node_cost, m_weight, p_node_new );

    RRTNode* p_min_node = p_nearest_node;

    for( list<RRTNode*>::iterator it = near_nodes.begin(); it != near_nodes.end(); it++ ) {
        RRTNode* p_near_node = (*it);
        if ( true == mp_parent->_is_obstacle_free(p_near_node->m_pos, p_node_new->m_pos) ) {
            vector<double> cost_temp(m_objective_num, 0.0);
            vector<double> cost_delta(m_objective_num, 0.0);
            mp_parent->calc_cost(p_near_node->m_pos, p_node_new->m_pos, cost_delta);
            for( unsigned int k=0; k < m_objective_num; k++ ) {
                cost_temp[k] = p_near_node->m_cost[k] + cost_delta[k];
            }
            double fitness = mp_parent->calc_fitness(cost_temp, m_weight, p_node_new);
            if ( fitness < min_new_node_fitness ) {
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
    if( mp_parent ) {
        std::list<KDNode2D> near_nodes = mp_parent->find_near( m_goal );
        double min_total_fitness = std::numeric_limits<double>::max();
        double min_delta_fitness = 0.0;
        double min_delta_cost[m_objective_num];
        RRTNode * p_min_prev_node = NULL;
        for( std::list<KDNode2D>::iterator it=near_nodes.begin();
            it != near_nodes.end(); it++ ) {
            KDNode2D kd_node = (*it);
            int index = m_index + m_objective_num;
            RRTNode* p_node = kd_node.m_node_list[index];

            if( mp_parent->_is_obstacle_free( p_node->m_pos, m_goal ) ) {
                vector<double> new_delta_cost(m_objective_num, 0.0);
                mp_parent->calc_cost( p_node->m_pos, m_goal, new_delta_cost );
                double new_delta_fitness = mp_parent->calc_fitness( new_delta_cost, m_weight, m_goal );
                double new_total_fitness = p_node->m_fitness + new_delta_fitness;
                if ( new_total_fitness < min_total_fitness ) {
                    p_min_prev_node = p_node;
                    min_delta_fitness = new_delta_fitness;
                    min_total_fitness = new_total_fitness;
                    for( unsigned int k=0; k<m_objective_num; k++ ) {
                        min_delta_cost[k] = new_delta_cost[k];
                    }
                }
            }
        }
        p_closest_node = p_min_prev_node;
        delta_fitness = min_delta_fitness;
        for( unsigned int k=0; k<m_objective_num; k++ ) {
            delta_cost[k] = min_delta_cost[k];
        }
    }
    return p_closest_node;
}
