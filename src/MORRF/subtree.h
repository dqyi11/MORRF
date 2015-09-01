#ifndef SUBTREE_H
#define SUBTREE_H

#include "KDTree2D.h"
#include <vector>
#include <list>

class MORRF;

class RRTNode {
public:
    RRTNode( POS2D pos, int objective_num );

    bool operator==( const RRTNode &other );

    int m_objective_num;
    double * mp_cost;
    double m_fitness;
    RRTNode * mp_parent;
    POS2D m_pos;

    std::list<RRTNode*> m_child_nodes;
};

class Path {
public:
    Path( POS2D start, POS2D goal, int objectiveNum );
    ~Path();

    int m_objective_num;
    double * mp_cost;
    double * mp_weight;
    double m_fitness;
    POS2D m_start;
    POS2D m_goal;
    std::vector<POS2D> m_waypoints;
};

class RRTree {
public:
    enum TREE_TYPE{ SUBPROBLEM, REFERENCE };
    RRTree( MORRF* parent, int objective_num, double * p_weight );

    RRTNode* init( POS2D start, POS2D goal );
    RRTNode* create_new_node( POS2D pos );
    bool remove_edge( RRTNode* p_node_p, RRTNode* p_node_c );
    bool has_edge( RRTNode* p_node_p, RRTNode* p_node_c );
    bool add_edge( RRTNode* p_node_p, RRTNode* p_node_c );

    std::list<RRTNode*> find_all_children( RRTNode* pNode );

    virtual void attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes ) = 0;
    virtual void rewire_near_nodes( RRTNode* p_node_new, std::list<RRTNode*> near_nodes ) = 0;
    virtual RRTNode * get_closet_to_goal( double * p_delta_cost, double& delta_fitness ) = 0;

    bool is_structure_correct();
    bool are_all_nodes_tractable();
    bool are_all_nodes_fitness_positive();
    RRTNode* find_ancestor( RRTNode* p_node );

    Path* find_path();


    TREE_TYPE m_type;
    int m_index;
    int m_objective_num;

    POS2D m_start;
    POS2D m_goal;

    MORRF* mp_parent;
    RRTNode * mp_root;
    double * mp_weight;

    std::list<RRTNode*> m_nodes;
};

class ReferenceTree : public RRTree {
public:
    ReferenceTree( MORRF* parent, int objective_num, int index );
    ~ReferenceTree();

    virtual void attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes );
    virtual void rewire_near_nodes( RRTNode* p_node_new, std::list<RRTNode*> near_nodes );
    virtual RRTNode * get_closet_to_goal( double * p_delta_cost, double& delta_fitness );

    Path* find_path();
protected:
    void update_fitness_to_children( RRTNode* pNode, double delta_fitness );
};

class SubproblemTree : public RRTree {
public:
    SubproblemTree( MORRF* parent, int objective_num, double * p_weight, int index );
    ~SubproblemTree();

    virtual void attach_new_node( RRTNode* p_node_new, RRTNode* p_nearest_node, std::list<RRTNode*> near_nodes );
    virtual void rewire_near_nodes( RRTNode* p_node_new, std::list<RRTNode*> near_nodes );
    virtual RRTNode * get_closet_to_goal( double * p_delta_cost, double& delta_fitness );
protected:
    void update_cost_to_children(RRTNode* p_node, double* p_delta_cost);

};

inline RRTNode* get_ancestor( RRTNode * p_node ) {
    if( NULL == p_node ) {
        return NULL;
    }
    if( NULL == p_node->mp_parent ) {
        return p_node;
    }
    else {
        return get_ancestor( p_node->mp_parent );
    }
}

inline void get_parent_node_list( RRTNode * p_node, std::list<RRTNode*>& path ) {
    if( p_node==NULL ) {
        return;
    }
    path.push_back( p_node );
    get_parent_node_list( p_node->mp_parent, path );
    return;
}

#endif // SUBTREE_H