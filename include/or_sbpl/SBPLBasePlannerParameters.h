#ifndef SBPL_BASE_PLANNER_PARAMETERS_H_
#define SBPL_BASE_PLANNER_PARAMETERS_H_

#include <openrave-core.h>
#include <openrave/planner.h>

namespace or_sbpl {

    class SBPLPlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters {

    public:
        SBPLPlannerParameters();
        
        int getNumAngles() const { return _numangles; }
        double getCellsize() const { return _cellsize; }



    };

}

#endif
