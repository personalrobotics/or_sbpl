#ifndef SBPL_BASE_PLANNER_H_
#define SBPL_BASE_PLANNER_H_

#include <boost/shared_ptr.hpp>

#include <sbpl/config.h>
#include <sbpl/planners/planner.h>

#include <or_sbpl/SBPLBasePlannerEnvironment.h>

#include <openrave/openrave.h>
#include <openrave/planner.h>

namespace or_sbpl {

    typedef boost::shared_ptr<SBPLPlanner> SBPLPlannerPtr;

    class SBPLBasePlanner : public OpenRAVE::PlannerBase  {

    public:
        SBPLBasePlanner(OpenRAVE::EnvironmentBasePtr penv);
        virtual ~SBPLBasePlanner();

        virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params);
        virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);
        virtual OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);
        virtual PlannerParametersConstPtr GetParameters() const { return _params; }
        
    private:

	void AddWaypoint(OpenRAVE::TrajectoryBasePtr ptraj, const OpenRAVE::ConfigurationSpecification &config_spec,
			 const double &x, const double &y, const double &theta,
			 const double &dx, const double &dy, const double &dtheta) const;


        OpenRAVE::EnvironmentBasePtr _orenv;
        OpenRAVE::RobotBasePtr _robot;
        PlannerParametersConstPtr _params;
        SBPLPlannerPtr _planner;
        SBPLBasePlannerEnvironmentPtr _env;

        bool _initialized;
    };
    
    typedef boost::shared_ptr<SBPLBasePlanner> SBPLBasePlannerPtr;

}
    
#endif
