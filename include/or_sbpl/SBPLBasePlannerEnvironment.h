#ifndef SBPL_BASE_PLANNER_ENVIRONMENT_H_
#define SBPL_BASE_PLANNER_ENVIRONMENT_H_

#include <sbpl/discrete_space_information/environment_navxythetalat.h>

#include <openrave/openrave.h>

namespace or_sbpl {

    class SBPLBasePlannerEnvironment : public EnvironmentNAVXYTHETALAT {
    public:
    SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot) : EnvironmentNAVXYTHETALAT(), _robot(robot) {}
        ~SBPLBasePlannerEnvironment(){}
        
    private:
        OpenRAVE::RobotBasePtr _robot;                
    };

    typedef boost::shared_ptr<SBPLBasePlannerEnvironment> SBPLBasePlannerEnvironmentPtr;

}

#endif
