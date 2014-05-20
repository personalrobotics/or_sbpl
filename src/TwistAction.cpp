
#include <or_sbpl/TwistAction.h>
#include <math.h>

#include <boost/math/constants/constants.hpp>

using namespace or_sbpl;
namespace bmc = boost::math::constants;

TwistAction::TwistAction() : _dx(0.0), _dy(0.0), _dtheta(0.0), _duration(0.0) {
    setName("twist");
    setWeight(1.0);

}

TwistAction::TwistAction(const double &dx, const double &dy, const double &dtheta, 
                         const double &duration, const double &weight) :
    _dx(dx), _dy(dy), _dtheta(dtheta), _duration(duration) {

    setName("twist");
    setWeight(weight);
}
        

bool TwistAction::apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const {

    bool valid = false;
    std::vector<WorldCoordinate> intermediates = applyWithIntermediates(wc, robot);
    if(intermediates.size() > 0){
        final_wc = intermediates.back();
        valid = true;
    }

    return valid;
}

std::vector<WorldCoordinate> TwistAction::applyWithIntermediates(const WorldCoordinate &wc,
                                                                 const OpenRAVE::RobotBasePtr &robot) const {

    // Create a robot state saver
    OpenRAVE::RobotBase::RobotStateSaver rStateSaver(robot);

    // Initialize the list of intermediate points
    std::vector<WorldCoordinate> intermediates;

    // Now step through the action, checking for collision along the way
    double timestep = 0.05;
    int steps = static_cast<int>(ceil(_duration/timestep));
    bool valid = true;
    WorldCoordinate wc_current(wc);
    for(int step = 0; (step < steps) & valid; step++){

        // Step the action        
        wc_current.theta += _dtheta*timestep;
        wc_current.x += _dx*timestep;
        wc_current.y += _dy*timestep;

        // Put the robot in the resulting pose
        OpenRAVE::Transform trans = wc.toTransform();
        robot->SetTransform(trans);

        // Check for collision and break out if needed
        bool incollision = robot->GetEnv()->CheckCollision(robot);
        valid = !incollision;

        if(valid){
            WorldCoordinate inter(wc_current);
            intermediates.push_back(inter);
        }
    }

    // Restore state
    rStateSaver.Restore();

    return intermediates;

}
