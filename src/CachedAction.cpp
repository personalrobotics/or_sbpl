#include <or_sbpl/CachedAction.h>
#include <boost/foreach.hpp>

using namespace or_sbpl;

CachedAction::CachedAction(){}

CachedAction::CachedAction(const std::vector<WorldCoordinate> &pts) :
    _pts(pts) {
    setName("cached");
}

bool CachedAction::apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const {
    
    bool valid = false;
    std::vector<WorldCoordinate> intermediates = applyWithIntermediates(wc, robot);
    if(intermediates.size() > 0){
        final_wc = intermediates.back();
        valid = true;
    }

    return valid;
}

std::vector<WorldCoordinate> CachedAction::applyWithIntermediates(const WorldCoordinate &wc,
                                                                  const OpenRAVE::RobotBasePtr &robot) const {

    // Create a robot state saver
    OpenRAVE::RobotBase::RobotStateSaver rStateSaver(robot);

    // Initialize the list of intermediate points
    std::vector<WorldCoordinate> intermediates;
    bool valid = true;
    BOOST_FOREACH(WorldCoordinate offset, _pts){
        
        // Move to the current offset within the action
        WorldCoordinate wc_current(wc);
        wc_current.x += offset.x;
        wc_current.x += offset.y;
        wc_current.theta += offset.theta;

        // Put the robot in the resulting pose
        OpenRAVE::Transform transform = wc_current.toTransform();
        robot->SetTransform(transform);

        // Check for collision and break out if needed
        bool incollision = robot->GetEnv()->CheckCollision(robot);
        valid = !incollision;
        
        if(valid){
            WorldCoordinate inter(wc_current);
            intermediates.push_back(inter);
        }else{
            break;
        }
                
    }
    
    // Restore state
    rStateSaver.Restore();

    return intermediates;
}

void operator >> (const YAML::Node& node,
                  boost::unordered_map<int,
                  std::vector<CachedAction> >& actions){
    actions.clear();
    for(YAML::Iterator field = node.begin(); field != node.end(); ++field){
        std::string key;
        field.first() >> key;
        const YAML::Node& value = field.second();
        if(key == "actions"){
            for(YAML::Iterator it = value.begin(); it != value.end(); ++it){
                int angle;
                std::vector<CachedAction> angle_actions;
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
    }
}

void operator >> (const YAML::Node& node, std::vector<CachedAction>& actions){
    actions.resize(node.size());
    size_t ii = 0;
    for(YAML::Iterator it = node.begin(); it != node.end(); ++it, ++ii){
        *it >> actions[ii];
        //std::cout << actions[ii]._pts[1].x << " " << actions[ii]._pts[1].y << std::endl;
    }
}

void operator >> (const YAML::Node& node, CachedAction& action){
    std::vector<WorldCoordinate> pts;
    pts.clear();
    
    for(YAML::Iterator it = node.begin(); it != node.end(); ++it){
        YAML::Iterator xyz = it->begin();
        WorldCoordinate action_coord;
        (*xyz) >> action_coord.x;
        ++xyz;
        (*xyz) >> action_coord.y;
        ++xyz;
        (*xyz) >> action_coord.theta;
        pts.push_back(action_coord);
        //std::cout << action_coord.x << " " << action_coord.y << " " << action_coord.theta << std::endl;
    }
    //std::cout << "A" << pts[3].x << " " << pts[3].y << " " << pts[3].theta << std::endl;
    action = CachedAction(pts);
    //std::cout << action._pts[3].x << " " << action._pts[3].y << " " << action._pts[3].theta << std::endl;
}
