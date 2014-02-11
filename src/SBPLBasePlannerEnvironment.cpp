#include <or_sbpl/SBPLBasePlannerEnvironment.h>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

using namespace or_sbpl; 

namespace bmc = boost::math::constants;

SBPLBasePlannerEnvironment::SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot) 
  : _robot(robot) {


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

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set start to grid coordinate: %s", gc.toString().c_str());
    int idx = GridCoordinateToStateIndex(gc);
    
    if( idx == INVALID_INDEX ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The start state %s is invalid.", gc.toString().c_str() );
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

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Set start to id: %d", _start);

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

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set goal to grid coordinate: %s\n", gc.toString().c_str());
    int idx = GridCoordinateToStateIndex(gc);
    
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

void SBPLBasePlannerEnvironment::ConvertStateIDPathIntoXYThetaPath(const std::vector<int> &state_ids,
                                                                   std::vector<sbpl_xy_theta_pt_t> &path) const {

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
    double euc_dist_m = hypot(wFrom.x - wTo.x, wFrom.y - wTo.y);
    return (int)(euc_dist_m*1000); //millimeters


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
            valid = env->CheckCollision(_robot);
        }

        if( valid ) {
            GridCoordinate gc_final = WorldCoordinateToGridCoordinate(wc_current);
            int state_idx = GridCoordinateToStateIndex(gc_final);

            if(state_idx != INVALID_INDEX){
                // Action propagatin led to a valid state

                std::map<int, int>::iterator it = StateIndex2StateIdTable.find(state_idx);
                int state_id;
                if( it == StateIndex2StateIdTable.end() ){
                    state_id = CreateState(gc);
                }else{
                    state_id = it->second;
                }
                SuccIDV->push_back(state_id);

                double euc_dist_m = hypot(wc.x - wc_current.x, wc.y - wc_current.y);
                CostV->push_back(euc_dist_m * 1000); // millimeters
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
        SBPL_FPRINTF(fOut, "World: X=%0.3f, Y=%0.3f, Theta=%0.3f", wc.x, wc.y, wc.theta);
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
    
    retCoord.x = gcoord.x*_cellsize + (_cellsize/2.0);
    retCoord.y = gcoord.y*_cellsize + (_cellsize/2.0);
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
    if( wcoord.x >= 0 ){
        retCoord.x = (int)(wcoord.x/_cellsize);
    }else{
        retCoord.x = (int)((wcoord.x/_cellsize) - 1);
    }

    if( wcoord.y >= 0 ){
        retCoord.y = (int)(wcoord.y/_cellsize);
    }else{
        retCoord.y = (int)((wcoord.y/_cellsize) - 1);
    }

    retCoord.theta = (int)(wcoord.theta/_anglesize);
    
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
