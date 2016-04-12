#ifndef MORRF_H
#define MORRF_H

#include "subtree.h"

typedef double (*COST_FUNC_PTR)(POS2D, POS2D, int**, void*);

class MORRFNode {
public:
    MORRFNode( POS2D pos ) {
        m_pos = pos;
    }

    POS2D m_pos;
    std::vector< RRTNode*> m_nodes;
};

class MORRF {
public:
    enum MORRF_TYPE{ WEIGHTED_SUM, TCHEBYCHEFF, BOUNDARY_INTERSACTION };
    MORRF( unsigned int width, unsigned int height, unsigned int objective_num, unsigned int subproblem_num, unsigned int segment_length, MORRF_TYPE type=WEIGHTED_SUM );
    ~MORRF();

    void add_funcs( std::vector<COST_FUNC_PTR> funcs, std::vector<int**> fitnessDistributions );

    void init(POS2D start, POS2D goal);

    void load_map( int **pp_map );
    POS2D sampling();
    POS2D steer( POS2D pos_a, POS2D pos_b );
    void extend();

    KDNode2D find_nearest( POS2D pos );
    std::list<KDNode2D> find_near( POS2D pos );

    bool _is_obstacle_free( POS2D pos_a, POS2D pos_b );
    bool _is_in_obstacle( POS2D pos );
    bool _contains( POS2D pos );

    bool calc_cost( POS2D& pos_a, POS2D& pos_b, std::vector<double>& cost );
    double calc_kth_cost( POS2D& pos_a, POS2D& pos_b, unsigned int k );
    double calc_fitness( std::vector<double>& cost, std::vector<double>& weight, RRTNode* node );
    double calc_fitness( std::vector<double>& cost, std::vector<double>& weight, POS2D& pos );

    bool get_utopia_reference_vector( POS2D& pos, std::vector<double>& utopia );
    bool get_utopia_reference_vector( RRTNode* p_node, std::vector<double>& utopia );

    int get_sampling_width() { return _sampling_width; }
    int get_sampling_height() { return _sampling_height; }

    void set_obstacle_info(int ** pp_obstacle) { _pp_map_info = pp_obstacle; }

    int get_current_iteration() { return _current_iteration; }

    ReferenceTree* get_reference_tree( unsigned int k );
    SubproblemTree* get_subproblem_tree( unsigned int m );

    std::vector<Path*> get_paths();

    int** get_map_info() { return _pp_map_info; }

    void dump_map_info( std::string filename );
    void dump_weights( std::string filename );

    float calc_fitness_by_weighted_sum( std::vector<double>& cost, std::vector<double>& weight );
    float calc_fitness_by_tchebycheff( std::vector<double>& cost, std::vector<double>& weight, std::vector<double>& utopia_reference );
    float calc_fitness_by_boundary_intersection( std::vector<double>& cost, std::vector<double>& weight, std::vector<double>& utopia_reference );

    bool are_reference_structures_correct();
    bool are_subproblem_structures_correct();
    bool are_all_reference_nodes_tractable();
    bool are_all_subproblem_nodes_tractable();
    bool are_all_reference_nodes_fitness_positive();
    bool are_all_subproblem_nodes_fitness_positive();
    bool is_node_number_identical();
    bool is_ref_tree_min_cost();
    double get_ball_radius() { return _ball_radius; }
    bool update_path_cost( Path *p );

    void optimize();
protected:
    void _init_weights();
    void _deinit_weights();

private:
    int ** _pp_map_info;

    MORRF_TYPE _type;
    unsigned int _sampling_width;
    unsigned int _sampling_height;

    unsigned int _objective_num;
    unsigned int _subproblem_num;

    KDTree2D * _p_kd_tree;

    std::vector<COST_FUNC_PTR> _funcs;
    std::vector<int**> _fitness_distributions;

    std::vector< std::vector<double> > _weights;

    std::vector<SubproblemTree*> _subproblems;
    std::vector<ReferenceTree*> _references;

    double _range;
    double _ball_radius;
    double _segment_length;
    int _obs_check_resolution;

    std::list<MORRFNode*> _morrf_nodes;

    double _theta;
    int _current_iteration;
};

#endif // MORRF_H
