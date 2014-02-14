#include <or_sbpl/SBPLBasePlanner.h>
#include <or_sbpl/TwistAction.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/utils.h>

using namespace or_sbpl;

SBPLBasePlanner::SBPLBasePlanner(OpenRAVE::EnvironmentBasePtr penv) :
    OpenRAVE::PlannerBase(penv), _orenv(penv), _initialized(false) {

}

SBPLBasePlanner::~SBPLBasePlanner() {

}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params) {

    if(!_initialized){
        _robot = robot;
        _params = params;
        _env = boost::make_shared<SBPLBasePlannerEnvironment>(robot);

        // Parse the extra parameters
        std::stringstream extra_stream;
        extra_stream << params->_sExtraParameters;
        int numangles = 0;
        double cellsize = 0.0;
        EnvironmentExtents extents;
        std::vector<ActionPtr> actions;
        while(extra_stream.good()) {
            
            std::string key;
            extra_stream >> key;
            
            if( key == "numangles" ){
                extra_stream >> numangles;
                RAVELOG_INFO("[SBPLBasePlannerParameters] Set numangles to %d\n", numangles);
            }else if( key == "cellsize" ) {
                extra_stream >> cellsize;
                RAVELOG_INFO("[SBPLBasePlannerParameters] Set cellsize to %0.3f\n", cellsize);
            }else if( key == "extents" ){
                extra_stream >> extents.xmin >> extents.xmax >> extents.ymin >> extents.ymax;
                RAVELOG_INFO("[SBPLBasePlannerParameters] Setting extents to [ %0.3f, %0.3f, %0.3f, %0.3f ]\n", 
                              extents.xmin, extents.xmax, extents.ymin, extents.ymax);
            }else if( key == "action" ) {
                double dx, dtheta, duration;
                extra_stream >> dx >> dtheta >> duration;
                ActionPtr action = boost::make_shared<TwistAction>(dx, dtheta, duration);
                RAVELOG_INFO("[SBPLBasePlannerParameters] Adding action: dx - %0.3f, dtheta - %0.3f, duration - %0.3f\n", dx, dtheta, duration);
                actions.push_back(action);
            }else{
                RAVELOG_WARN("[SBPLBasePlanner] Unrecognized parameters: %s\n", key.c_str());
            }
                
        }

        if( extra_stream.fail() ){
            return false;
        }
        
        _env->Initialize(cellsize, extents, numangles, actions);
        _planner = boost::make_shared<ARAPlanner>(_env.get(), true);

        _initialized = true;
    }

    return _initialized;

}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input) {


}



OpenRAVE::PlannerStatus SBPLBasePlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {


    RAVELOG_INFO("[SBPLBasePlanner] Begin PlanPath\n");

    /* Setup the start point for the plan */
    try{
        
        std::vector<OpenRAVE::dReal> start_vals(3);
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
        // TODO: Replace this max time parameter
        std::vector<int> plan;
        int path_cost;
        int solved = _planner->replan(10.0, &plan, &path_cost);
        RAVELOG_INFO("[SBPLBasePlanner] Solved? %d\n", solved);
        if( solved ){

            /* Write out the trajectory to return back to the caller */
            OpenRAVE::ConfigurationSpecification config_spec = OpenRAVE::RaveGetAffineConfigurationSpecification(OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis,
                                                                                                                 _robot, "linear");
            int time_group = config_spec.AddDeltaTimeGroup();
            OpenRAVE::ConfigurationSpecification::Group affine_group = config_spec.GetGroupFromName("affine_transform");
            int affine_offset = affine_group.offset;

            ptraj->Init(config_spec);
            std::vector<sbpl_xy_theta_pt_t> xyth_path;
            _env->ConvertStateIDPathIntoXYThetaPath(plan, xyth_path);

            for(unsigned int idx=0; idx < xyth_path.size(); idx++){

                // Grab this point in the planned path
                sbpl_xy_theta_pt_t pt = xyth_path[idx];

                // Create a trajectory point
                std::vector<double> point;
                point.resize(config_spec.GetDOF());

                // Manually set the timing
                point[time_group] = idx;

                // Set the affine values
                OpenRAVE::RaveTransformMatrix<double> rot;
                rot.rotfrommat(OpenRAVE::RaveCos(pt.theta), -OpenRAVE::RaveSin(pt.theta), 0.,
                               OpenRAVE::RaveSin(pt.theta),  OpenRAVE::RaveCos(pt.theta), 0.,
                               0., 0., 1.);
                               
                OpenRAVE::RaveVector<double> trans(pt.x, pt.y, 0.0); //TODO: Is this z correct?
                OpenRAVE::RaveTransform<double> transform(OpenRAVE::geometry::quatFromMatrix(rot), trans);
                    
                OpenRAVE::RaveGetAffineDOFValuesFromTransform(point.begin() + affine_offset, 
                                                              transform, 
                                                              OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis);
                
                // Insert the point
                ptraj->Insert(idx, point, true);

                return OpenRAVE::PS_HasSolution;
            }

        }else{
            RAVELOG_ERROR("[SBPLBasePlanner] SBPL unable to find solution in allocated time\n");
            return OpenRAVE::PS_Failed;
        }

    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while planning\n");
        return OpenRAVE::PS_Failed;
    }


    
    
}
