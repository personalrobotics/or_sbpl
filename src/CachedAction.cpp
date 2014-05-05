#include <or_sbpl/CachedAction.h>
#include <boost/foreach.hpp>

using namespace or_sbpl;

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
