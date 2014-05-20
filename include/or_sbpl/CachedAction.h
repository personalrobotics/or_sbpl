#ifndef SBPL_CACHED_ACTION_H_
#define SBPL_CACHED_ACTION_H_

#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <boost/unordered_map.hpp>
#include "yaml-cpp/yaml.h"

namespace or_sbpl {

    /**
     * An action that is parameterized as a cached list of points that make up the action
     */
    class CachedAction : public Action {

    public: 
	/**
	 * Constructor.
	 */
        CachedAction();

        /**
         * Constructor. 
         *
         * @param pts The list of points that comprise this action
	 * @param weight The weight to apply to the action cost
         */
        CachedAction(const std::vector<WorldCoordinate> &pts, const double &weight);

        /**
         * Applies the action, checking for collision at each cached point
         *
         * @param wc The pose to start action propagation from
         * @param robot The robot to apply the twist to
         * @param final_wc The world coordinate that the robot reaches by applying the action
         * @return True if the action was successful. False otherwise.
         */
        virtual bool apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const;
 
        /**
         * Applies the action
         *
         * @param wc The pose to start action propagation from
         * @param robot The robot to apply the twist to
         * @return A list of all valid poses visited during execution of the action
         */
        virtual std::vector<WorldCoordinate> applyWithIntermediates(const WorldCoordinate &wc,
                                                                    const OpenRAVE::RobotBasePtr &robot) const;
	
	/**
	 * @return The list of points in the action
	 */
	virtual std::vector<WorldCoordinate> getPoints() const { return _pts; }

    private:
        std::vector<WorldCoordinate> _pts;

    };
    typedef boost::shared_ptr<CachedAction> CachedActionPtr;
}

#endif
