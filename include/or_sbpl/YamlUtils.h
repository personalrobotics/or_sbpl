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

    inline void operator >> (const YAML::Node& node, ActionPtr& action){
        std::vector<WorldCoordinate> pts;
        pts.clear();

	double weight = 1.0;
	
        for(YAML::Iterator it = node.begin(); it != node.end(); ++it){
	    std::string primitive_key;
	    it.first() >> primitive_key;
	    const YAML::Node& primitive_value = it.second();
	    if(primitive_key == "poses"){
		// Read in the pose list
		for(YAML::Iterator pose_it = primitive_value.begin(); pose_it != primitive_value.end(); ++pose_it){
		    YAML::Iterator xyz = pose_it->begin();
		    WorldCoordinate action_coord;
		    *pose_it >> action_coord;
		    pts.push_back(action_coord);
		}
	    }else if(primitive_key == "weight"){
		primitive_value >> weight;
	    }
        }

        action = boost::make_shared<CachedAction>(pts, weight);
    }

    inline void operator >> (const YAML::Node& value,
                             ActionList &actions) {
        actions.clear();
        for(YAML::Iterator it = value.begin(); it != value.end(); ++it){
            unsigned int angle;
            std::vector<ActionPtr> angle_actions;
            for(YAML::Iterator ii = (*it).begin(); ii != (*it).end(); ++ii){
                std::string action_key;
                ii.first() >> action_key;
                const YAML::Node& action_value = ii.second();
                if(action_key == "angle"){
                    action_value >> angle;
                }
                else if(action_key == "primitives"){
                    action_value >> angle_actions;
                }
            }
            actions[angle] = angle_actions;
        }
        
    }

    inline void operator >> (const YAML::Node& node, std::vector<ActionPtr>& actions){
        actions.resize(node.size());
        size_t ii = 0;
        for(YAML::Iterator it = node.begin(); it != node.end(); ++it, ++ii){
            *it >> actions[ii];
        }
    }



}
