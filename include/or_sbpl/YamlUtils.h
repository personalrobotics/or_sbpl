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
    
        for(YAML::Iterator it = node.begin(); it != node.end(); ++it){
            YAML::Iterator xyz = it->begin();
            WorldCoordinate action_coord;
            *it >> action_coord;
            pts.push_back(action_coord);
//            std::cout << action_coord.x << " " << action_coord.y << " " << action_coord.theta << std::endl;
        }
        //std::cout << "A" << pts[3].x << " " << pts[3].y << " " << pts[3].theta << std::endl;
        action = boost::make_shared<CachedAction>(pts);
        //std::cout << action._pts[3].x << " " << action._pts[3].y << " " << action._pts[3].theta << std::endl;
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
                else if(action_key == "poses"){
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
            //std::cout << actions[ii]._pts[1].x << " " << actions[ii]._pts[1].y << std::endl;
        }
    }



}
