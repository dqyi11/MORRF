#ifndef MORRF_H
#define MORRF_H

#include "KDTree2D.h"
#include "subtree.h"

typedef double (*COST_FUNC_PTR)(POS2D, POS2D, int**, void*);


class MORRF {
public:
    enum MORRF_TYPE{ WEIGHTED_SUM, TCHEBYCHEFF, BOUNDARY_INTERSACTION };
    MORRF( int width, int height, int objective_num, int subproblem_num, int segment_length, MORRF_TYPE type=WEIGHTED_SUM );
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

    bool calc_cost( POS2D& pos_a, POS2D& pos_b, double * p_cost );
    double calc_cost( POS2D& pos_a, POS2D& pos_b, int k );
    double calc_fitness( double * p_cost, double * p_weight, POS2D& pos );

    bool get_utopia_reference_vector( POS2D& pos, double * p_utopia );

    int get_sampling_width() { return _sampling_width; }
    int get_sampling_height() { return _sampling_height; }

    void set_obstacle_info(int ** pp_obstacle) { _pp_map_info = pp_obstacle; }

    int get_current_iteration() { return _current_iteration; }

    ReferenceTree* get_reference_tree( int k );
    SubproblemTree* get_subproblem_tree( int m );

    std::vector<Path*> get_paths();

    int** get_map_info() { return _pp_map_info; }

    void dump_map_info( std::string filename );
    void dump_weights( std::string filename );

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
    int _sampling_width;
    int _sampling_height;

    int _objective_num;
    int _subproblem_num;

    KDTree2D * _p_kd_tree;

    std::vector<COST_FUNC_PTR> _funcs;
    std::vector<int**> _fitness_distributions;

    double** _pp_weights;

    std::vector<SubproblemTree*> _subproblems;
    std::vector<ReferenceTree*> _references;

    double _range;
    double _ball_radius;
    double _segment_length;
    int _obs_check_resolution;

    double _theta;
    int _current_iteration;
};

#endif // MORRF_H
