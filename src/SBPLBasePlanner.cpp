#include <or_sbpl/SBPLBasePlanner.h>
#include <or_sbpl/TwistAction.h>
#include <or_sbpl/YamlUtils.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <sbpl/planners/araplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/utils.h>

#include <yaml-cpp/yaml.h>

using namespace or_sbpl;

SBPLBasePlanner::SBPLBasePlanner(OpenRAVE::EnvironmentBasePtr penv) :
    OpenRAVE::PlannerBase(penv), _orenv(penv), _initialized(false), _maxtime(10.0),
    _epsinit(5.0), _epsdec(0.2), _return_first(false) {

}

SBPLBasePlanner::~SBPLBasePlanner() {

}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params) {

    _robot = robot;
    _params = params;
    _env = boost::make_shared<SBPLBasePlannerEnvironment>(robot);
    
    // Parse the extra parameters
    std::stringstream extra_stream;
    extra_stream << params->_sExtraParameters;

    double linear_weight;
    double theta_weight;
    double cellsize = 0.0;
    int numangles = 0;

    EnvironmentExtents extents;    
    ActionList actions;

#ifdef YAMLCPP_NEWAPI
    YAML::Node doc = YAML::Load(extra_stream);

    linear_weight = doc["linear_weight"].as<double>();
    theta_weight = doc["theta_weight"].as<double>();
    doc["extents"] >> extents;
    cellsize = doc["cellsize"].as<double>();
    numangles = doc["numangles"].as<int>();
    doc["actions"] >> actions;
    _maxtime = doc["timelimit"].as<double>();
#else
    // Parse the extra parameters as yaml
    YAML::Parser parser(extra_stream);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    RAVELOG_INFO("[SBPLBasePlanner] Parsing\n");

    doc["linear_weight"] >> linear_weight;
    doc["theta_weight"] >> theta_weight;
    doc["extents"] >> extents;
    doc["cellsize"] >> cellsize;
    doc["numangles"] >> numangles;
    doc["actions"] >> actions;
    doc["timelimit"] >> _maxtime;
    
    if (YAML::Node const *init_eps = doc.FindValue("initial_eps")) {
        *init_eps >> _epsinit;
        RAVELOG_INFO("[SBPLBasePlanner] Initial epsilon: %0.3f\n", _epsinit);
    }


    if (YAML::Node const *dec_eps = doc.FindValue("dec_eps")) {
        *dec_eps >> _epsdec;
        RAVELOG_INFO("[SBPLBasePlanner] Epsilon decrement: %0.3f\n", _epsdec);
    }

    if (YAML::Node const *return_first = doc.FindValue("return_first")) {
        int rfirst;
        *return_first >> rfirst;
        _return_first = (rfirst == 1);
    }
#endif

    RAVELOG_INFO("[SBPLBasePlanner] Theta weight: %0.3f\n", theta_weight);
    RAVELOG_INFO("[SBPLBasePlanner] Cellsize: %0.3f\n", cellsize);
    RAVELOG_INFO("[SBPLBasePlanner] Num angles: %d\n", numangles);
    RAVELOG_INFO("[SBPLBasePlanner] Time limit: %0.3f\n", _maxtime);
    RAVELOG_INFO("[SBPLBasePlanner] Return first: %s\n", (_return_first ? "True" : "False") );

    _env->Initialize(cellsize, extents, numangles, actions, linear_weight, theta_weight);
    _planner = boost::make_shared<ARAPlanner>(_env.get(), true);

    return true;

}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input) {


}



OpenRAVE::PlannerStatus SBPLBasePlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {


    RAVELOG_INFO("[SBPLBasePlanner] Begin PlanPath\n");

    /* Setup the start point for the plan */
    std::vector<OpenRAVE::dReal> start_vals(3);
    try{
        
        OpenRAVE::RaveGetAffineDOFValuesFromTransform(start_vals.begin(),
                                                      _robot->GetTransform(),
                                                      OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis);
        int start_id = _env->SetStart(start_vals[0], start_vals[1], start_vals[2]);

        if( start_id < 0 || _planner->set_start(start_id) == 0){
            RAVELOG_ERROR("[SBPLBasePlanner] Failed to set start state\n");
            return OpenRAVE::PS_Failed;
        }


    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while setting the start state\n");
        return OpenRAVE::PS_Failed;
    }
    
    /* Setup the goal point for the plan */
    try{

        // Get the configuration specification and the goal from the parameters and turn
        // it into an x,y,theta
        std::vector<OpenRAVE::dReal> goal_vals;

        // NOTE: This is the correct thing to do, but openrave doesn't support an affine config spec for the parameters
        //_params->_configurationspecification.ExtractAffineValues(goal_vals.begin(),
        //                                                         _params->vgoalconfig.begin(),
        //                                                         _robot,
        //                                                         OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis);
        goal_vals = _params->vgoalconfig;

        if(goal_vals.size() != 3){
            RAVELOG_ERROR("[SBPLBasePlanner] Unable to extract goal of appropriate size.\n");
            return OpenRAVE::PS_Failed;
        }

        int goal_id = _env->SetGoal(goal_vals[0], goal_vals[1], goal_vals[2]); 
        if( goal_id < 0 || _planner->set_goal(goal_id) == 0){
            RAVELOG_ERROR("[SBPLBasePlanner] Failed to set goal state\n");
            return OpenRAVE::PS_Failed;
        }

    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while setting the goal state\n");
        return OpenRAVE::PS_Failed;
    }

    /* Attempt to plan */
    try {
        std::vector<int> plan;
        int path_cost;
        ReplanParams rparams(_maxtime);
        rparams.initial_eps = _epsinit;
        rparams.dec_eps = _epsdec;
        rparams.return_first_solution = _return_first;
        rparams.max_time = _maxtime;

        int solved = _planner->replan(&plan, rparams, &path_cost);
        RAVELOG_INFO("[SBPLBasePlanner] Solved? %d\n", solved);
        if( solved ){

            /* Write out the trajectory to return back to the caller */
            OpenRAVE::ConfigurationSpecification config_spec = OpenRAVE::RaveGetAffineConfigurationSpecification(OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis,
                                                                                                                 _robot, "linear");
            config_spec.AddDerivativeGroups(1, true);  //velocity group, add delta time group
            ptraj->Init(config_spec);
            std::vector<PlannedWaypointPtr> xyth_path;

            // Add the start pose to the trajectory - this is not in the returned path
            //  by default
            AddWaypoint(ptraj, config_spec,
                        start_vals[0], start_vals[1], start_vals[2]);

            // Now add the rest of the path
            _env->ConvertStateIDPathIntoWaypointPath(plan, xyth_path);
            for(unsigned int idx=0; idx < xyth_path.size(); idx++){

                // Grab this point in the planned path
                PlannedWaypointPtr pt = xyth_path[idx];

                // Convert it to a trajectory waypoint
                AddWaypoint(ptraj, config_spec, 
                            pt->coord.x, pt->coord.y, pt->coord.theta);
            }

            return OpenRAVE::PS_HasSolution;

        }else{
            RAVELOG_ERROR("[SBPLBasePlanner] SBPL unable to find solution in allocated time\n");
            return OpenRAVE::PS_Failed;
        }

    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while planning\n");
        return OpenRAVE::PS_Failed;
    }
    
}

/*
 * Creates a waypoint and adds it to the trajectory
 * 
 * @param ptraj The trajectory to append the waypoint to
 * @param config_spec The configuration specification for the trajectory
 * @param t The delta time value
 * @param x The x location
 * @param y The y location
 * @param theta The orientation
 */
void SBPLBasePlanner::AddWaypoint(OpenRAVE::TrajectoryBasePtr ptraj, const OpenRAVE::ConfigurationSpecification &config_spec, 
                                  const double &x, const double &y, const double &theta) const {

    // Create a trajectory point
    std::vector<double> point;
    point.resize(config_spec.GetDOF());

    // Get group offsets
    OpenRAVE::ConfigurationSpecification::Group dt_group = config_spec.GetGroupFromName("deltatime");
    int time_group = dt_group.offset;
    OpenRAVE::ConfigurationSpecification::Group affine_group = config_spec.GetGroupFromName("affine_transform");
    int affine_offset = affine_group.offset;

    // Manually set the timing
    int idx = ptraj->GetNumWaypoints();
    point[time_group] = idx;

    // Set the affine values
    OpenRAVE::RaveTransformMatrix<double> rot;
    rot.rotfrommat(OpenRAVE::RaveCos(theta), -OpenRAVE::RaveSin(theta), 0.,
                   OpenRAVE::RaveSin(theta),  OpenRAVE::RaveCos(theta), 0.,
                   0., 0., 1.);
                               
    OpenRAVE::RaveVector<double> trans(x, y, 0.0); //TODO: Is this z correct?
    OpenRAVE::RaveTransform<double> transform(OpenRAVE::geometry::quatFromMatrix(rot), trans);
    
    OpenRAVE::RaveGetAffineDOFValuesFromTransform(point.begin() + affine_offset, 
                                                  transform, 
                                                  OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis);
 
    // Insert the point
    ptraj->Insert(idx, point, true);
    
}
