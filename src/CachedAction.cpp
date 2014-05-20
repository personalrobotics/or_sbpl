#include <or_sbpl/CachedAction.h>
#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace or_sbpl;

CachedAction::CachedAction() {
    setWeight(1.0);
}

CachedAction::CachedAction(const std::vector<WorldCoordinate> &pts, const double &weight) :
    _pts(pts) {
    setName("cached");
    setWeight(weight);
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
        wc_current.y += offset.y;
        wc_current.theta = offset.theta;

        // Put the robot in the resulting pose
        OpenRAVE::Transform transform = wc_current.toTransform();
        robot->SetTransform(transform);

        // Check for collision and break out if needed
        bool incollision = robot->GetEnv()->CheckCollision(robot);
        valid = !incollision;
        
        if(valid){
            intermediates.push_back(wc_current);
        }else{
            intermediates.clear();
            break;
        }
                
    }
    
    // Restore state
    rStateSaver.Restore();

    return intermediates;
}
