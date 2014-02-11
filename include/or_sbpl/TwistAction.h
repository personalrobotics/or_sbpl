#ifndef SBPL_TWIST_ACTION_H_
#define SBPL_TWIST_ACTION_H_

#include <or_sbpl/SBPLBasePlannerTypes.h>

namespace or_sbpl {

    class TwistAction : public Action {
    
    public:
        TwistAction();
        TwistAction(const double &dx, const double &dtheta, const double &duration);
        
        virtual WorldCoordinate apply(const WorldCoordinate &wc, const double &timestep) const;
        
        double dx;
        double dtheta;

    };


}

#endif
