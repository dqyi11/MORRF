#include "morrf.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <limits>

#define OBSTACLE_THRESHOLD 200

MORRF::MORRF(int width, int height, int objective_num, int subproblem_num, int segmentLength, MORRF_TYPE type) {
    _sampling_width = width;
    _sampling_height = height;
    _objective_num = objective_num;
    _subproblem_num = subproblem_num;
    _type = type;

    _p_kd_tree = new KDTree2D(std::ptr_fun(tac));

    _range = (_sampling_width > _sampling_height) ? _sampling_width:_sampling_height;
    _ball_radius = _range;
    _obs_check_resolution = 1;
    _current_iteration = 0;
    _segment_length = segmentLength;

    _pp_weights = NULL;
    _theta = 5;

    _pp_map_info = new int*[_sampling_width];
    for(int i=0;i<_sampling_width;i++) {
        _pp_map_info[i] = new int[_sampling_height];
    }
}

MORRF::~MORRF() {
    _deinit_weights();

    if(_p_kd_tree) {
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

    if (_objective_num == 2) {
        for(int i=0;i<_subproblem_num;i++) {
            _pp_weights[i] = new double[_objective_num];
            _pp_weights[i][0] = (double)(i+1) / (double)(_subproblem_num+2);
            _pp_weights[i][1] = (double)(_subproblem_num-i+1) / (double)(_subproblem_num+2);
        }
    }
    else {
        for(int i=0;i<_subproblem_num;i++) {
            _pp_weights[i] = new double[_objective_num];
            for(int j=0;j<_objective_num;j++) {
                _pp_weights[i][j] = (double)rand()/RAND_MAX;
            }
        }
    }
}

void MORRF::_deinit_weights() {
    if(_pp_weights) {
        int size = sizeof(_pp_weights)/sizeof(double*);
        for(int i=0;i<size;i++) {
            if(_pp_weights[i]) {
                delete _pp_weights[i];
                _pp_weights[i] = NULL;
            }
        }
    }
}

void MORRF::init(POS2D start, POS2D goal) {
    _init_weights();

    KDNode2D root(start);
    for(int k=0;k<_objective_num;k++) {
        ReferenceTree * pRefTree = new ReferenceTree(this, _objective_num, k);
        RRTNode * pRootNode = pRefTree->init(start, goal);
        root.mNodeList.push_back(pRootNode);
        _references.push_back(pRefTree);
    }

    for(int m=0;m<_subproblem_num;m++) {
        SubproblemTree * pSubTree = new SubproblemTree(this, _objective_num, _pp_weights[m], m);
        RRTNode * pRootNode = pSubTree->init(start, goal);
        root.mNodeList.push_back(pRootNode);
        _subproblems.push_back(pSubTree);
    }
    _p_kd_tree->insert(root);
    _current_iteration = 0;
}

void MORRF::load_map(int **map)
{
    for(int i=0;i<_sampling_width;i++)
    {
        for(int j=0;j<_sampling_height;j++)
        {
            _pp_map_info[i][j] = map[i][j];
        }
    }
}

POS2D MORRF::sampling()
{
    double x = rand();
    double y = rand();
    x = x * ((double)(_sampling_width)/RAND_MAX);
    y = y * ((double)(_sampling_height)/RAND_MAX);

    POS2D m(x,y);
    return m;
}

POS2D MORRF::steer(POS2D pos_a, POS2D pos_b)
{
    POS2D new_pos(pos_a[0], pos_a[1]);
    double delta[2];
    delta[0] = pos_a[0] - pos_b[0];
    delta[1] = pos_a[1] - pos_b[1];
    double delta_len = std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]);

    if (delta_len > _segment_length)
    {
        double scale = _segment_length / delta_len;
        delta[0] = delta[0] * scale;
        delta[1] = delta[1] * scale;

        new_pos.setX( pos_b[0]+delta[0] );
        new_pos.setY( pos_b[1]+delta[1] );
    }
    return new_pos;
}

bool MORRF::_is_in_obstacle(POS2D pos)
{
    int x = (int)pos[0];
    int y = (int)pos[1];
    if( _pp_map_info[x][y] < 255)
        return true;
    return false;
}


bool MORRF::_is_obstacle_free(POS2D pos_a, POS2D pos_b)
{
    if (pos_a == pos_b)
        return true;
    int x_dist = pos_a[0] - pos_b[0];
    int y_dist = pos_a[1] - pos_b[1];
    if (fabs(x_dist) > fabs(y_dist))
    {
        double k = (double)y_dist/ x_dist;
        int startX, endX, startY;
        if (pos_a[0] < pos_b[0])
        {
            startX = pos_a[0];
            endX = pos_b[0];
            startY = pos_a[1];
        }
        else
        {
            startX = pos_b[0];
            endX = pos_a[0];
            startY = pos_b[1];
        }
        for (int coordX = startX; coordX < endX + _obs_check_resolution ; coordX+=_obs_check_resolution)
        {
            int coordY = (int)(k*(coordX-startX)+startY);
            if (coordY >= _sampling_height || coordX >= _sampling_width) break;
            if ( _pp_map_info[coordX][coordY] < OBSTACLE_THRESHOLD )
            {
                return false;
            }
        }
    }
    else
    {
        double k = (double)x_dist/ y_dist;
        int startY, endY, startX;
        if (pos_a[1] < pos_b[1])
        {
            startY = pos_a[1];
            endY = pos_b[1];
            startX = pos_a[0];
        }
        else
        {
            startY = pos_b[1];
            endY = pos_a[1];
            startX = pos_b[0];
        }
        for (int coordY = startY; coordY < endY + _obs_check_resolution ; coordY+=_obs_check_resolution)
        {
            int coordX = (int)(k*(coordY-startY)+startX);
            if (coordY >= _sampling_height || coordX >= _sampling_width) break;
            if ( _pp_map_info[coordX][coordY] < OBSTACLE_THRESHOLD )
            {
                return false;
            }
        }
    }
    return true;
}

void MORRF::extend()
{
    bool node_inserted = false;
    while(false==node_inserted)
    {
        POS2D rndPos = sampling();
        KDNode2D nearest_node = find_nearest(rndPos);

        POS2D new_pos = steer(rndPos, nearest_node);

        if(true == _contains(new_pos))
        {
            continue;
        }
        if( true==_is_in_obstacle(new_pos) )
        {
            continue;
        }

        if(true==_is_obstacle_free(nearest_node, new_pos))
        {
            std::list<KDNode2D> near_nodes = find_near(new_pos);
            KDNode2D new_node(new_pos);

            // create new nodes of reference trees
            for(int k=0;k<_objective_num;k++)
            {
                RRTNode * pNewRefNode = _references[k]->createNewNode(new_pos);
                new_node.mNodeList.push_back(pNewRefNode);
            }

            // create new nodes of subproblem trees
            for (int m=0;m<_subproblem_num;m++)
            {
                RRTNode * pNewSubNode = _subproblems[m]->createNewNode(new_pos);
                new_node.mNodeList.push_back(pNewSubNode);
            }

            _p_kd_tree->insert(new_node);
            node_inserted = true;

            // attach new node to reference trees
            // rewire near nodes of reference trees
            for (int k=0;k<_objective_num;k++)
            {
                // std::cout << "@ " << k << std::endl;
                int index = k;
                RRTNode* pNearestRefNode = nearest_node.mNodeList[index];
                RRTNode* pNewRefNode = new_node.mNodeList[index];
                std::list<RRTNode*> nearRefNodes;
                nearRefNodes.clear();
                for(std::list<KDNode2D>::iterator itr = near_nodes.begin();
                    itr != near_nodes.end(); itr++)
                {
                    KDNode2D kd_node = (*itr);
                    RRTNode* pRefNode = kd_node.mNodeList[index];
                    nearRefNodes.push_back(pRefNode);
                }

                _references[k]->attach_new_node(pNewRefNode, pNearestRefNode, nearRefNodes);
                _references[k]->rewire_near_nodes(new_node.mNodeList[index], nearRefNodes);
            }

            // attach new nodes to subproblem trees
            // rewire near nodes of subproblem trees
            for(int m=0;m<_subproblem_num;m++)
            {
                // std::cout << "@ " << m+mObjectiveNum << std::endl;
                int index = m+_objective_num;
                RRTNode* pNearestSubNode = nearest_node.mNodeList[index];
                RRTNode* pNewSubNode = new_node.mNodeList[index];
                std::list<RRTNode*> nearSubNodes;
                nearSubNodes.clear();
                for(std::list<KDNode2D>::iterator its = near_nodes.begin();
                    its != near_nodes.end(); its++)
                {
                    KDNode2D kd_node = (*its);
                    RRTNode* pSubNode = kd_node.mNodeList[index];
                    nearSubNodes.push_back(pSubNode);
                }

                _subproblems[m]->attach_new_node(pNewSubNode, pNearestSubNode, nearSubNodes);
                _subproblems[m]->rewire_near_nodes(new_node.mNodeList[index], nearSubNodes);
            }
        }
    }
    _current_iteration++;
}

KDNode2D MORRF::find_nearest(POS2D pos)
{
    KDNode2D node(pos);

    std::pair<KDTree2D::const_iterator,double> found = _p_kd_tree->find_nearest(node);
    KDNode2D near_node = *found.first;
    return near_node;
}

std::list<KDNode2D> MORRF::find_near(POS2D pos)
{
    std::list<KDNode2D> near_list;
    KDNode2D node(pos);

    int numVertices = _p_kd_tree->size();
    int numDimensions = 2;
    _ball_radius = _range * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );

    _p_kd_tree->find_within_range(node, _ball_radius, std::back_inserter(near_list));

    return near_list;
}


bool MORRF::_contains(POS2D pos)
{
    if(_p_kd_tree)
    {
        KDNode2D node(pos[0], pos[1]);
        KDTree2D::const_iterator it = _p_kd_tree->find(node);
        if(it!=_p_kd_tree->end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

bool MORRF::calc_cost(POS2D& pos_a, POS2D& pos_b, double * p_cost)
{
    if (p_cost==NULL)
    {
        return false;
    }
    for(int k=0;k<_objective_num;k++)
    {
        p_cost[k] = calc_cost(pos_a, pos_b, k);
    }
    return true;
}

double MORRF::calc_cost(POS2D& pos_a, POS2D& pos_b, int k)
{
    int dimension[2];
    dimension[0] = _sampling_width;
    dimension[1] = _sampling_height;
    return _funcs[k](pos_a, pos_b, _fitness_distributions[k], dimension);
}

double MORRF::calc_fitness(double * p_cost, double * p_weight, POS2D& pos)
{
    double fitness = 0.0;
    if(p_cost == NULL || p_weight==NULL)
    {
        return fitness;
    }
    if(_type==MORRF::WEIGHTED_SUM)
    {
        for(int k=0;k<_objective_num;k++)
        {
            fitness += p_cost[k] * p_weight[k];
        }
    }
    else if(_type==MORRF::TCHEBYCHEFF) {
        double p_utopia[_objective_num];
        if(true == getUtopiaReferenceVector(pos, p_utopia)) {
            for(int k=0;k<_objective_num;k++) {
               double weighted_dist = p_weight[k] * fabs(p_cost[k] - p_utopia[k]);
               if (weighted_dist > fitness) {
                   fitness = weighted_dist;
               }
            }
        }
    }
    else {
        double p_utopia[_objective_num];
        if(true == getUtopiaReferenceVector(pos, p_utopia)) {
            double d1 = 0.0, d2 = 0.0;
            for(int k=0;k<_objective_num;k++) {
               double weighted_dist = p_weight[k] * (p_cost[k] - p_utopia[k]);
               d1 += weighted_dist;
            }
            d1 = fabs(d1);
            double vectorD2[_objective_num];
            for(int k=0;k<_objective_num;k++) {
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

bool MORRF::getUtopiaReferenceVector(POS2D& pos, double * p_utopia)
{
    if ( p_utopia==NULL ) {
        return false;
    }
    KDNode2D ref_node = find_nearest(pos);
    if(ref_node.mNodeList.size()<_objective_num) {
        return false;
    }

    for(int k=0;k<_objective_num;k++) {
        RRTNode* pRRTNode = ref_node.mNodeList[k];
        p_utopia[k] = pRRTNode->mFitness;
    }
    return true;
}

ReferenceTree* MORRF::get_reference_tree(int k)
{
    if(k<0 || k>=_objective_num) {
        return NULL;
    }
    return _references[k];
}

SubproblemTree* MORRF::get_subproblem_tree( int m ) {
    if(m<0 || m>=_subproblem_num) {
        return NULL;
    }
    return _subproblems[m];
}

void MORRF::dump_map_info( std::string filename ) {
    std::ofstream mapInfoFile;
    mapInfoFile.open(filename.c_str());
    if( _pp_map_info ) {
        for(int j=0;j<_sampling_height;j++) {
            for(int i=0;i<_sampling_width;i++) {
                mapInfoFile << _pp_map_info[i][j] << " ";
            }
            mapInfoFile << std::endl;
        }
    }
    mapInfoFile.close();
}

bool MORRF::areReferenceStructuresCorrect() {
    for(std::vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree) {
            if( false==pRefTree->is_structure_correct() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::areSubproblemStructuresCorrect() {
    for(std::vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* pSubTree = (*it);
        if(pSubTree) {
            if(false==pSubTree->is_structure_correct()) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::areAllReferenceNodesTractable() {
    for(std::vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree) {
            if( false==pRefTree->are_all_nodes_tractable() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::areAllSubproblemNodesTractable() {
    for(std::vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* pSubTree = (*it);
        if(pSubTree) {
            if( false==pSubTree->are_all_nodes_tractable() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::areAllReferenceNodesFitnessPositive() {
    for(std::vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree) {
            if(false==pRefTree->are_all_nodes_fitness_positive()) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::areAllSubproblemNodesFitnessPositive() {
    for(std::vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* pSubTree = (*it);
        if(pSubTree) {
            if( false==pSubTree->are_all_nodes_fitness_positive() ) {
                return false;
            }
        }
    }
    return true;
}

bool MORRF::isNodeNumberIdentical() {
    int ref_num = _references[0]->mNodes.size();

    for(std::vector<ReferenceTree*>::iterator it=_references.begin();it!=_references.end();it++) {
        ReferenceTree* pRefTree = (*it);
        if(pRefTree) {
            int num = pRefTree->mNodes.size();
            if(num != ref_num) {
                return false;
            }
        }
    }
    for(std::vector<SubproblemTree*>::iterator it=_subproblems.begin();it!=_subproblems.end();it++) {
        SubproblemTree* pSubTree = (*it);
        if(pSubTree) {
            int num = pSubTree->mNodes.size();
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
            p->mpCost[k] = 0.0;
        }
        for(int i=0;i<p->mWaypoints.size()-1;i++)
        {
            POS2D pos_a = p->mWaypoints[i];
            POS2D pos_b = p->mWaypoints[i+1];
            double deltaCost[_objective_num];
            calc_cost(pos_a, pos_b, deltaCost);

            for(int k=0;k<_objective_num;k++)
            {
                p->mpCost[k] += deltaCost[k];
            }
        }        
        return true;
    }
    return false;
}

bool MORRF::isRefTreeMinCost() {
    if(_p_kd_tree) {
        for(KDTree2D::const_iterator it = _p_kd_tree->begin(); it!= _p_kd_tree->end(); it++) {
            KDNode2D node = (*it);
            double minCost[_objective_num];
            for(int k=0;k<_objective_num;k++) {
                minCost[k] = std::numeric_limits<double>::max();
            }
            for(int i=_objective_num; i<node.mNodeList.size(); i++) {
                RRTNode* pNode = node.mNodeList[i];
                for(int k=0;k<_objective_num;k++) {
                    if(pNode->mpCost[k] < minCost[k]) {
                        minCost[k] = pNode->mpCost[k];
                    }
                }
            }
            for(int k=0;k<_objective_num;k++) {
                RRTNode* pRefNode = node.mNodeList[k];
                if(pRefNode->mFitness > minCost[k]) {
                    return false;
                }
            }
        }
    }
    return true;
}
