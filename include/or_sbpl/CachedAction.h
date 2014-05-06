#ifndef SBPL_CACHED_ACTION_H_
#define SBPL_CACHED_ACTION_H_

#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <boost/unordered_map.hpp>
#include "yaml-cpp/yaml.h"

namespace or_sbpl {

    class CachedAction : public Action {

    public: 
        /**
         * Constructor. 
         *
         * @param pts The list of points that comprise this action
         */
        CachedAction();
        CachedAction(const std::vector<WorldCoordinate> &pts);

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

    private:
        std::vector<WorldCoordinate> _pts;

    };
    typedef boost::shared_ptr<CachedAction> CachedActionPtr;
}
void operator >> (const YAML::Node& node,
                  boost::unordered_map<int,
                  std::vector<or_sbpl::CachedAction> >&);

void operator >> (const YAML::Node& node, std::vector<or_sbpl::CachedAction>&);

void operator >> (const YAML::Node& node, or_sbpl::CachedAction&);

#endif
