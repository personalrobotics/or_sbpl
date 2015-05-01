#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <or_sbpl/CachedAction.h>
#include <yaml-cpp/yaml.h>
#include <boost/make_shared.hpp>

namespace or_sbpl {

#ifdef YAMLCPP_NEWAPI
    typedef YAML::iterator yaml_iterator;
    typedef YAML::const_iterator yaml_const_iterator;

    template <typename T>
    void yaml_get(const YAML::Node &node, T &value)
    {
        value = node.as<T>();
    }

#else
    typedef YAML::Iterator yaml_iterator;
    typedef YAML::Iterator yaml_const_iterator;

    template <typename T>
    void yaml_get(const YAML::Node &node, T &value)
    {
        node >> value;
    }
#endif

    inline void operator >> (const YAML::Node& node, EnvironmentExtents& extents) {
#ifdef YAMLCPP_NEWAPI
        extents.xmin = node[0].as<double>();
        extents.xmax = node[1].as<double>();
        extents.ymin = node[2].as<double>();
        extents.ymax = node[3].as<double>();
#else
        node[0] >> extents.xmin;
        node[1] >> extents.xmax;
        node[2] >> extents.ymin;
        node[3] >> extents.ymax;
#endif
    }

    inline void operator >> (const YAML::Node& node, WorldCoordinate& wc) {
#ifdef YAMLCPP_NEWAPI
        wc.x = node[0].as<double>();
        wc.y = node[1].as<double>();
        wc.theta = node[2].as<double>();
#else
        node[0] >> wc.x;
        node[1] >> wc.y;
        node[2] >> wc.theta;
#endif
    }

    inline void operator >> (const YAML::Node& node, ActionPtr& action){
        std::vector<WorldCoordinate> pts;
        pts.clear();

        double weight = 1.0;

        for(yaml_const_iterator it = node.begin(); it != node.end(); ++it){
            std::string primitive_key;

#ifdef YAMLCPP_NEWAPI
            primitive_key = it->first.as<std::string>();
            const YAML::Node& primitive_value = it->second;
#else
            it.first() >> primitive_key;
            const YAML::Node& primitive_value = it.second();
#endif

            if(primitive_key == "poses"){
                // Read in the pose list
                for(yaml_const_iterator pose_it = primitive_value.begin(); pose_it != primitive_value.end(); ++pose_it){
                    yaml_const_iterator xyz = pose_it->begin();
                    WorldCoordinate action_coord;
                    *pose_it >> action_coord;
                    pts.push_back(action_coord);
                }
            }else if(primitive_key == "weight"){
#ifdef YAMLCPP_NEWAPI
                weight = primitive_value.as<double>();
#else
                primitive_value >> weight;
#endif
            }
        }

        action = boost::make_shared<CachedAction>(pts, weight);
    }

    inline void operator >> (const YAML::Node& node, std::vector<ActionPtr>& actions){
        actions.resize(node.size());

        size_t ii = 0;
        for(yaml_const_iterator it = node.begin(); it != node.end(); ++it, ++ii){
            *it >> actions[ii];
        }
    }

    inline void operator >> (const YAML::Node& value,
                             ActionList &actions) {
        actions.clear();
        for(yaml_const_iterator it = value.begin(); it != value.end(); ++it){
            unsigned int angle;
            std::vector<ActionPtr> angle_actions;
            for(yaml_const_iterator ii = (*it).begin(); ii != (*it).end(); ++ii){
                std::string action_key;
#ifdef YAMLCPP_NEWAPI
                action_key = ii->first.as<std::string>();
                const YAML::Node& action_value = ii->second;
                if(action_key == "angle"){
                    angle = action_value.as<unsigned int>();
                }
                else if(action_key == "primitives"){
                    action_value >> angle_actions;
                }
#else
                ii.first() >> action_key;
                const YAML::Node& action_value = ii.second();
                if(action_key == "angle"){
                    action_value >> angle;
                }
                else if(action_key == "primitives"){
                    action_value >> angle_actions;
                }
#endif
            }
            actions[angle] = angle_actions;
        }
    }
}
