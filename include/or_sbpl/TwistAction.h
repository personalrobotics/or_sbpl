#ifndef SBPL_TWIST_ACTION_H_
#define SBPL_TWIST_ACTION_H_

#include <or_sbpl/SBPLBasePlannerTypes.h>

namespace or_sbpl {

    /**
     * An action that is parameterized as a twist
     */
    class TwistAction : public Action {
    
    public:
        /**
         * Constructor. All twist parameters are defaulted to zero.
         */
        TwistAction();

        /**
         * Constructor.
         *
         * @param dx The translational velocity in world x
         * @param dy The translational velocity in world y
         * @param dtheta The rotational velocity
         * @param duration The duration to apply the twist
	 * @param weight The weight to apply to the cost of this twist
         */
        TwistAction(const double &dx, const double &dy, const double &dtheta, const double &duration, const double &weight);
        
        /** 
         * Destructor
         */
        virtual ~TwistAction() {}

        /**
         * Applies the twist represented by this class to the given world coordinate.
         *
         * @param wc The pose to start action propagation from
         * @param robot The robot to apply the twist to
         * @param final_wc The world coordinate that the robot reaches by applying the action
         * @return True if the action was successful. False otherwise.
         */
        virtual bool apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const;

        /**
         * Applies the twist represented by this class. 
         *
         * @param wc The pose to start action propagation from
         * @param robot The robot to apply the twist to
         * @return A list of all valid poses visited during execution of the action
         */
        virtual std::vector<WorldCoordinate> applyWithIntermediates(const WorldCoordinate &wc,
                                                                    const OpenRAVE::RobotBasePtr &robot) const;

    private:
        double _dx;
        double _dy;
        double _dtheta;
        double _duration;
    };


}

#endif
