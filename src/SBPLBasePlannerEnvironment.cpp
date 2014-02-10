#include <or_sbpl/SBPLBasePlannerEnvironment.h>

using namespace or_sbpl; 

SBPLBasePlannerEnvironment::SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot) 
  : _robot(robot) {


}

bool SBPLBasePlannerEnvironment::InitializeEnv(const char* sEnvFile) {

}

bool SBPLBasePlannerEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg) {

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

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set goal to grid coordinate: %s", gc.toString().c_str());
    int idx = GridCoordinateToStateIndex(gc);
    
    if( idx == INVALID_INDEX ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The goal state %s is invalid.", gc.toString().c_str() );
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

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Set goal to id: %d", _goal);

    return state_id;
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
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] ERROR in GetFromToHeuristic: FromStateID %d is illegal", FromStateID);
        throw new SBPL_Exception();
    }
    
    if(ToStateID >= (int) StateId2CoordTable.size()){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] ERROR in GetFromToHeuristic: ToStateID %d is illegal", ToStateID);
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
 
void SBPLBasePlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){

}

void SBPLBasePlannerEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){

}

void SBPLBasePlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state){

}
 
void SBPLBasePlannerEnvironment::SetAllPreds(CMDPSTATE* state){

}

int SBPLBasePlannerEnvironment::SizeofCreatedEnv(){

}

void SBPLBasePlannerEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut){

}

void SBPLBasePlannerEnvironment::PrintEnv_Config(FILE* fOut){

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
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The state index %d is invalid.", stateidx);
        throw new SBPL_Exception();
    }
    
    std::map<int, int>::const_iterator it = StateIndex2StateIdTable.find(stateidx);
    if( it == StateIndex2StateIdTable.end() ){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The state index %d maps to an uninitialized state id.", stateidx);
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
