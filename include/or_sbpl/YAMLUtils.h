#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <or_sbpl/CachedAction.h>
#include <yaml-cpp/yaml.h>
#include <boost/make_shared.hpp>

namespace or_sbpl {

    inline void operator >> (const YAML::Node& node, EnvironmentExtents& extents) {
	node[0] >> extents.xmin;
	node[1] >> extents.xmax;
	node[2] >> extents.ymin;
	node[3] >> extents.ymax;
    }

    inline void operator >> (const YAML::Node& node, WorldCoordinate& wc) {
	node[0] >> wc.x;
	node[1] >> wc.y;
	node[2] >> wc.theta;
    }
    
    inline void operator >> (const YAML::Node& actions, std::map<unsigned int, std::vector<ActionPtr> > &actionmap) {
	
	// Iterate each of the angles
	for(unsigned int angidx = 0; angidx < actions.size(); angidx++){
	    const YAML::Node& angle_node = actions[angidx];
	    unsigned int angle;
	    angle_node["angle"] >> angle;
	    
	    
	    const YAML::Node& poses = angle_node["poses"];
	    // Iterate each action within the angle
	    std::vector<ActionPtr> action_list;
	    for(unsigned int aidx=0; aidx < poses.size(); aidx++){
		const YAML::Node& action = poses[aidx];
		
		// Iterate each coordinate within the action
		std::vector<WorldCoordinate> coords;
		for(unsigned int cidx=0; cidx < action.size(); cidx++){
		    WorldCoordinate wc;
		    action[cidx] >> wc;
		    coords.push_back(wc);
		}

		// Now create a new cached action
		CachedActionPtr a = boost::make_shared<CachedAction>(coords);
		action_list.push_back(a);
	    }

	    // Add the action list to he appropriate place in the dictionary
	    actionmap[angle] = action_list;
	}
    }
}
