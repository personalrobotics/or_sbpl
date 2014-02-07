#include <or_sbpl/SBPLBasePlanner.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <sbpl/planners/araplanner.h>

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

        // TODO: Initialize the environment

        _planner = boost::make_shared<ARAPlanner>(_env.get(), true);

        _initialized = true;
    }

    return _initialized;

}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input) {


}

OpenRAVE::PlannerStatus SBPLBasePlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {


    RAVELOG_INFO("[SBPLBasePlanner] Begin PlanPath");

    /* Setup the start point for the plan */
    try{
        
        OpenRAVE::Transform rpose = _robot->GetTransform();
        OpenRAVE::RaveTransformMatrix<double> mat;
        OpenRAVE::geometry::matrixFromQuat(mat, rpose.rot);
        int start_id = _env->SetStart(rpose.trans.x, rpose.trans.y, atan2(mat.rot(0,0), mat.rot(1,0))); //TODO: Double check
        
        //TODO: Check that this is a valid id

    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while setting the start state");
        return OpenRAVE::PS_Failed;
    }
    
    /* Setup the goal point for the plan */
    try{

        // Get the configuration specification and the goal from the parameters and turn
        // it into an x,y,theta
        std::vector<OpenRAVE::dReal> goal_vals;
        _params->_configurationspecification.ExtractAffineValues(goal_vals.begin(),
                                                                 _params->vgoalconfig.begin(),
                                                                 _robot,
                                                                 OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis);
        BOOST_FOREACH(OpenRAVE::dReal v, goal_vals){
            std::cout << "Val: " << v << std::endl;
        }

        int goal_id = _env->SetGoal(goal_vals[0], goal_vals[1], goal_vals[2]); 

        // TODO: Check that this is a valid id

    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while setting the goal state");
        return OpenRAVE::PS_Failed;
    }

    /* Attempt to plan */
    try {
        // TODO: Replace this max time parameter
        std::vector<int> plan;
        int path_cost;
        int solved = _planner->replan(10.0, &plan, &path_cost);
        if( solved ){

            /* Write out the trajectory to return back to the caller */
            OpenRAVE::ConfigurationSpecification config_spec = OpenRAVE::RaveGetAffineConfigurationSpecification(OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis,
                                                                                                                 _robot, "linear");
            int time_group = config_spec.AddDeltaTimeGroup();
            OpenRAVE::ConfigurationSpecification::Group affine_group = config_spec.GetGroupFromName("affine_transform");
            int affine_offset = affine_group.offset;

            ptraj->Init(config_spec);
            std::vector<sbpl_xy_theta_pt_t> xyth_path;
            _env->ConvertStateIDPathintoXYThetaPath(&plan, &xyth_path);

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
            RAVELOG_ERROR("[SBPLBasePlanner] SBPL unable to find solution in allocated time");
            return OpenRAVE::PS_Failed;
        }

    }catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while planning");
        return OpenRAVE::PS_Failed;
    }


    
    
}
