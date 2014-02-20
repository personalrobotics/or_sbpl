#include <or_sbpl/SBPLBasePlannerEnvironment.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>

using namespace or_sbpl; 

namespace bmc = boost::math::constants;

SBPLBasePlannerEnvironment::SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot) 
    : _robot(robot), _timestep(0.05) {


}

/*
 * Initialize the environment from the given parameters
 *
 * @param cellsize The size of the grid cellsize
 * @param extents The extends of the environment
 * @param angles The number of angles to divide the space
 * @param actions The list of valid actions
 */
bool SBPLBasePlannerEnvironment::Initialize(const double &cellsize,
                                            const EnvironmentExtents &extents,
                                            const int &numangles,
                                            const std::vector<ActionPtr> &actions){

    // Setup environment attributes
    _cellsize = cellsize;

    _gridwidth = static_cast<int>(ceil((extents.xmax - extents.xmin)/_cellsize));
    _gridheight = static_cast<int>(ceil((extents.ymax - extents.ymin)/_cellsize));
    _extents = extents;

    // Angles
    _numangles = numangles;
    _anglesize = 2.0*bmc::pi<double>()/_numangles;

    // Actions
    _actions = actions;

    return true;
    
}

/*
 * Not implemented - we want to initialize from the OpenRAVE planner parameters
 */
bool SBPLBasePlannerEnvironment::InitializeEnv(const char* sEnvFile) {

    RAVELOG_ERROR("[SBPLBasePlannerEnvironment] InitializeEnv not implemented");
    throw new SBPL_Exception();
}

/*
 * Sets the start and the goal states in the MDPCfg 
 */
bool SBPLBasePlannerEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg) {

    MDPCfg->goalstateid = _goal;
    MDPCfg->startstateid = _start;

    return true;
}

/*
 * Sets the start state.
 *
 * @param x The x position of the start
 * @param y  The y position of the start
 * @param theta The yaw of the start
 * @return The associated state id
 */
int SBPLBasePlannerEnvironment::SetStart(const double &x, const double &y, const double &theta) {

    WorldCoordinate wc(x, y, theta);
    GridCoordinate gc = WorldCoordinateToGridCoordinate(wc);

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set start to grid coordinate: %s\n", gc.toString().c_str());
    int idx = GridCoordinateToStateIndex(gc);
    
    if( idx == INVALID_INDEX ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The start state %s is invalid.\n", gc.toString().c_str() );
        throw new SBPL_Exception();
    }

    std::map<int, int>::iterator it = StateIndex2StateIdTable.find(idx);
    int state_id;
    if(it == StateIndex2StateIdTable.end()){
        state_id = CreateState(gc);
    }else{
        state_id = it->second;
    }

    _start = state_id;

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Set start to id: %d\n", _start);

    return state_id;
}

/*
 * Sets the goal state.
 *
 * @param x The x position of the goal
 * @param y  The y position of the goal
 * @param theta The yaw of the goal
 * @return The associated state id
 */
int SBPLBasePlannerEnvironment::SetGoal(const double &x, const double &y, const double &theta) {

    WorldCoordinate wc(x, y, theta);
    GridCoordinate gc = WorldCoordinateToGridCoordinate(wc);

    int idx = GridCoordinateToStateIndex(gc);
    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set goal to grid coordinate %d: %s (%s)\n", idx, wc.toString().c_str(), gc.toString().c_str());    
    if( idx == INVALID_INDEX ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The goal state %s is invalid.\n", gc.toString().c_str() );
        throw new SBPL_Exception();
    }

    std::map<int, int>::iterator it = StateIndex2StateIdTable.find(idx);
    int state_id;
    if(it == StateIndex2StateIdTable.end()){
        state_id = CreateState(gc);
    }else{
        state_id = it->second;
    }

    _goal = state_id;

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Set goal to id: %d\n", _goal);

    return state_id;
}

/*
 * Returns x,y,theta-coords for each point along the path.
 * 
 *
 * @param state_ids The list of state ids that make up the path
 * @param path The converted path
 */
void SBPLBasePlannerEnvironment::ConvertStateIDPathIntoWaypointPath(const std::vector<int> &state_ids,
								    std::vector<PlannedWaypointPtr> &path) {


    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Begin ConvertStateIDPathIntoXYThetaPath\n");

    // clear out the vector just in case
    path.clear();

    // iterate through the path
    for(unsigned int pidx = 0; pidx < state_ids.size()-1; pidx++){
        
        // Grab the states that start and end this action
        int start_id = state_ids[pidx];
        int goal_id = state_ids[pidx+1];

        // Now search through all successors to find the best (least cost) one
        //   that leads to the goal
        std::vector<int> succ_ids;
        std::vector<int> costs;
        std::vector<ActionPtr> actions;
        
        GetSuccs(start_id, &succ_ids, &costs, &actions);
        
        int best_idx = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for(unsigned int idx=0; idx < succ_ids.size(); idx++){

            int succ_id = succ_ids[idx];
            if(succ_id == goal_id && costs[idx] <= best_cost){
                best_cost = costs[idx];
                best_idx = idx;
            }
        }

        // If we didn't find a successor something has gone terribly wrong, bail
        if(best_idx == -1){
            RAVELOG_ERROR("[SBPLBasePlannerEnvironment] Failed to reconstruct path.");
            throw new SBPL_Exception();
        }

        // Play the action forward, setting all the intermediate states
        ActionPtr a = actions[best_idx];
        int steps = static_cast<int>(ceil(a->getDuration()/_timestep));

        GridCoordinate gc = StateId2CoordTable[start_id];
        WorldCoordinate wc_current = GridCoordinateToWorldCoordinate(gc);
        WorldCoordinate wc_next;
        for(unsigned int step = 0; step < steps; step++){

            // Step the action
            wc_next = a->apply(wc_current, _timestep);

            // Add this pose to the pose list
	    PlannedWaypointPtr pt = boost::make_shared<PlannedWaypoint>(wc_next, a);
            path.push_back(pt);
            
            // Set this point to current and iterate
            wc_current = wc_next;
        }
    }

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Generated path of length %d\n", path.size());

    return;
}

/*
 * Returns the heuristic cost between the two states
 * In this case, distance in millimeters.
 *
 * @param FromStateID The state to start at
 * @param ToStateID The state to go to
 * @return Euclidean distance between FromStateID and ToStateID (in mm)
 */
int SBPLBasePlannerEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID){
    if(FromStateID >= (int) StateId2CoordTable.size()){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] ERROR in GetFromToHeuristic: FromStateID %d is illegal\n", FromStateID);
        throw new SBPL_Exception();
    }
    
    if(ToStateID >= (int) StateId2CoordTable.size()){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] ERROR in GetFromToHeuristic: ToStateID %d is illegal\n", ToStateID);
        throw new SBPL_Exception();
    }

    // Grab the correct world coordinate
    GridCoordinate gTo = StateId2CoordTable[ToStateID];
    WorldCoordinate wTo = GridCoordinateToWorldCoordinate(gTo);

    GridCoordinate gFrom = StateId2CoordTable[FromStateID];
    WorldCoordinate wFrom = GridCoordinateToWorldCoordinate(gFrom);

    // Calculate the euclidean distance
    double cost = ComputeCost(wFrom, wTo);
    return (int)(cost);


}

/*
 * Returns the heuristic cost between the two states
 * In this case, distance in millimeters.
 *
 * @param stateID The state to start at
 * @return Euclidean distance between the state and the goal
 */
int SBPLBasePlannerEnvironment::GetGoalHeuristic(int stateID){
    return GetFromToHeuristic(stateID, _goal);
}

/*
 * Returns the distance in millimeters between
 *  the given state and the start.
 *
 * @param stateID The state to start at
 * @return Euclidean distance between the state and the start
 */
int SBPLBasePlannerEnvironment::GetStartHeuristic(int stateID){
    return GetFromToHeuristic(stateID, _start);

}
 
/*
 * Returns a list of valid successor states.  Actions leading to these successors
 * have been checked for collision against the openrave environment.
 *
 * @param SourceStateID The state to get successors for
 * @param SuccIDV The list of valid successors
 * @param CostV The cost to move to each predecessor
 */
void SBPLBasePlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){

    std::vector<ActionPtr> ignored;
    GetSuccs(SourceStateID, SuccIDV, CostV, &ignored);
}
/*
 * Returns a list of valid successor states.  Actions leading to these successors
 * have been checked for collision against the openrave environment.
 *
 * @param SourceStateID The state to get successors for
 * @param SuccIDV The list of valid successors
 * @param CostV The cost to move to each predecessor
 * @param ActionV The action that moves to the predecessor
 */
void SBPLBasePlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<ActionPtr>* ActionV){

    SuccIDV->clear();
    CostV->clear();

    // Check validity of the state
    if( !IsValidStateId(SourceStateID) ){
        RAVELOG_ERROR("[SBPLBasePlanningEnvironment] Id %d is invalid state\n", SourceStateID);
        return;
    }

    // If this is the goal state, just return
    if( SourceStateID == _goal ){
        RAVELOG_INFO("[SBPLBasePlanningEnvironment] Expanded goal. Returning.\n");
        return;
    }


    // Convert to a world coordinate
    GridCoordinate gc = StateId2CoordTable[SourceStateID];
    WorldCoordinate wc = GridCoordinateToWorldCoordinate(gc);

    RAVELOG_DEBUG("[SBPLBasePlanningEnvironment] Expanding node %d: %s (%s)\n",
                 SourceStateID, wc.toString().c_str(), gc.toString().c_str());

    // Lock the environment
    OpenRAVE::EnvironmentBasePtr env = _robot->GetEnv();
    OpenRAVE::EnvironmentMutex::scoped_lock lock(env->GetMutex());

    // Create a robot state saver
    OpenRAVE::RobotBase::RobotStateSaver rStateSaver(_robot);

    // Now step through each of the actions
    BOOST_FOREACH(ActionPtr a, _actions){

        // Now step through the action, checking for collision along the way
        int steps = static_cast<int>(ceil(a->getDuration()/_timestep));
        bool valid = true;
        WorldCoordinate wc_next;
        WorldCoordinate wc_current = wc;
        for(unsigned int step = 0; step < steps & valid; step++){

            // Step the action
            wc_next = a->apply(wc_current, _timestep);

            // Put the robot in the resulting pose
            OpenRAVE::Transform trans = WorldCoordinateToTransform(wc_next);
            _robot->SetTransform(trans);

            // Check for collision and break out if needed
            bool incollision = env->CheckCollision(_robot);
            valid = !incollision;

            wc_current = wc_next;
        }

        if( valid ) {
            GridCoordinate gc_final = WorldCoordinateToGridCoordinate(wc_current);
            int state_idx = GridCoordinateToStateIndex(gc_final);

            if(state_idx != INVALID_INDEX){
                // Action propagatin led to a valid state

                std::map<int, int>::iterator it = StateIndex2StateIdTable.find(state_idx);
                int state_id;
                if( it == StateIndex2StateIdTable.end() ){
                    state_id = CreateState(gc_final);
                }else{
                    state_id = it->second;
                }

                RAVELOG_DEBUG("[SBPLBasePlannerEnvironment] Adding successor state %d (idx %d): %s (%s)\n", 
                              state_id, state_idx, wc_current.toString().c_str(), 
                              gc_final.toString().c_str());

                SuccIDV->push_back(state_id);

                double cost = ComputeCost(wc, wc_current);
                CostV->push_back(cost); 

                ActionV->push_back(a);
            }
        }

    }

    // Restore state
    rStateSaver.Restore();

}

void SBPLBasePlannerEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){

}

/*
 * Not implemented
 */
void SBPLBasePlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state){

    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] SetAllActionsandAllOutcomes is not implemented.\n");
    throw new SBPL_Exception();
}
 
/*
 * Not implemented
 */
void SBPLBasePlannerEnvironment::SetAllPreds(CMDPSTATE* state){

    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] SetAllPreds is not implemented.\n");
    throw new SBPL_Exception();
}

/*
 * @return The number of states currently in the environment
 */
int SBPLBasePlannerEnvironment::SizeofCreatedEnv(){
    return (int) StateId2CoordTable.size();
}

/*
 * Prints the state represented by the given id
 *
 * @param stateID The state to print
 * @param bVerbose If true, prints both world and grid coordinate, otherwise only gridcoord is printed
 * @param fOut The output location
 */
void SBPLBasePlannerEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut){

    if( stateID > StateId2CoordTable.size() ){
        RAVELOG_ERROR("[SBPLBasePlanningEnvironment] Invalid state id: %d.\n", stateID);
        throw new SBPL_Exception();
    }

    GridCoordinate gc = StateId2CoordTable[stateID];

    if(fOut == NULL){
        fOut = stdout;
    }

    SBPL_FPRINTF(fOut, "Grid: X=%d, Y=%d, Theta=%d", gc.x, gc.y, gc.theta);
    if(bVerbose){
        WorldCoordinate wc = GridCoordinateToWorldCoordinate(gc);
        SBPL_FPRINTF(fOut, " World: X=%0.3f, Y=%0.3f, Theta=%0.3f\n", wc.x, wc.y, wc.theta);
    }else{
        SBPL_FPRINTF(fOut, "\n");
    }
}

/*
 * Not implemented
 */
void SBPLBasePlannerEnvironment::PrintEnv_Config(FILE* fOut){

    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] PrintEnv_Config is not implemented.\n");
    throw new SBPL_Exception();
}

/**
 * Converts from a grid coordinate to a world coordinate
 *
 * @param stateid The id of the state to convert
 * @return The world coordinate described by the state
 */
WorldCoordinate SBPLBasePlannerEnvironment::GridCoordinateToWorldCoordinate(const GridCoordinate &gcoord) const {

    WorldCoordinate retCoord;
    
    retCoord.x = gcoord.x*_cellsize + (_cellsize/2.0) + _extents.xmin;
    retCoord.y = gcoord.y*_cellsize + (_cellsize/2.0) + _extents.ymin;
    retCoord.theta = gcoord.theta*_anglesize;

    return retCoord;

}
 
/**
 * Converts from a world coordinate to a grid coordinate
 *
 * @param coord The world coordinate to convert
 * @return The coordinate of the grid cell containing the world coordinate
 */
GridCoordinate SBPLBasePlannerEnvironment::WorldCoordinateToGridCoordinate(const WorldCoordinate &wcoord) const {

    
    GridCoordinate retCoord;
    double x = wcoord.x - _extents.xmin;
    if( x < 0 ){ x = 0.0; }
    retCoord.x = (int)(x/_cellsize);
    
    double y = wcoord.y - _extents.ymin;
    if( y < 0 ){ y = 0.0; }
    retCoord.y = (int)(y/_cellsize);
    
    double theta = wcoord.theta;
    while(theta < 0){
        theta += 2.0*bmc::pi<double>();
    }

    while(theta >= 2.0*bmc::pi<double>()){
        theta -= 2.0*bmc::pi<double>();
    }

    retCoord.theta = (int)(theta/_anglesize);
    
    return retCoord;

}

/**
 * Converts the state id to a grid coordinate
 *
 * @param stateid The state id to convert
 * @return The associated grid coordinate
 */
GridCoordinate SBPLBasePlannerEnvironment::StateIndexToGridCoordinate(unsigned int stateidx) const{

    if( stateidx >= StateIndex2StateIdTable.size() ){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The state index %d is invalid.\n", stateidx);
        throw new SBPL_Exception();
    }
    
    std::map<int, int>::const_iterator it = StateIndex2StateIdTable.find(stateidx);
    if( it == StateIndex2StateIdTable.end() ){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The state index %d maps to an uninitialized state id.\n", stateidx);
        throw new SBPL_Exception();
    }

    return StateId2CoordTable[it->second];
}

/**
 * Converts the grid coordinate to a state id
 *
 * @param coord The grid coordinate to convert
 * @return The state id associated with the grid coordinate
 */
int SBPLBasePlannerEnvironment::GridCoordinateToStateIndex(const GridCoordinate &coord) const{

    int retIdx = INVALID_INDEX;


    //check validity
    if(coord.x < 0 || coord.x >= _gridwidth || 
       coord.y < 0 || coord.y >= _gridheight ||
       coord.theta < 0 || coord.theta >= _numangles){
        return retIdx;
    }
     
    retIdx = coord.theta + coord.y * _numangles + coord.x * _numangles * _gridheight;

    return retIdx;
  
}

/*
 * Creates a state from the given coordinate
 *
 * @param coord The coordinate
 * @return The id of the created state
 */
int SBPLBasePlannerEnvironment::CreateState(const GridCoordinate &gc) {

    // get the index
    int state_idx = GridCoordinateToStateIndex(gc);

    if(state_idx == INVALID_INDEX){
        return UNINITIALIZED_ID;
    }

    // generate a new id and add the node
    int state_id = StateId2CoordTable.size();
    StateId2CoordTable.push_back(gc);
    StateIndex2StateIdTable[state_idx] = state_id;

    int *entry = new int[1];
    StateID2IndexMapping.push_back(entry);
    StateID2IndexMapping[state_id][0] = -1; // TODO: What is this? copied from SBPL code.

    return state_id;
}

/*
 * Checks a state id for validity
 *
 * @param state_id The id to check
 * @return True if the id is valid
 */
bool SBPLBasePlannerEnvironment::IsValidStateId(const int &state_id) const {
    
    if( state_id < 0 || state_id >= StateId2CoordTable.size() ){
        return false;
    }else{
        return true;
    }
}

/*
 * Converts a (x,y,theta) pose to a transform for the robot
 *
 * @param wcoord The pose to convert
 * @return The associated transform
 */
OpenRAVE::Transform SBPLBasePlannerEnvironment::WorldCoordinateToTransform(const WorldCoordinate &wcoord) const {


    // Rotation
    OpenRAVE::RaveTransformMatrix<double> R;
    R.rotfrommat(OpenRAVE::RaveCos(wcoord.theta), -OpenRAVE::RaveSin(wcoord.theta), 0.,
                 OpenRAVE::RaveSin(wcoord.theta),  OpenRAVE::RaveCos(wcoord.theta), 0.,
                 0., 0., 1.);

    // Translation
    OpenRAVE::RaveVector<double> t(wcoord.x, wcoord.y, 0.0); //TODO: Fix the z coord

    // Now put them together
    OpenRAVE::RaveTransform<double> transform(OpenRAVE::geometry::quatFromMatrix(R), t);
    return transform;

}

/*
 * Computes the cost of moving from one world coordinate to another
 * 
 * @param c1 The start world coordinate
 * @param c2 The end world coordinate
 * @return The cost
 */
double SBPLBasePlannerEnvironment::ComputeCost(const WorldCoordinate &c1, const WorldCoordinate &c2) const 
{

    double euc_dist_m = hypot(c1.x - c2.x, c1.y - c2.y);
    double angle_diff = c2.theta - c1.theta;
    angle_diff = fmod((angle_diff + bmc::pi<double>()), 2.0*bmc::pi<double>()) - bmc::pi<double>();
    return euc_dist_m*1000 + fabs(angle_diff)*10; //millimeters, milliradians
}
