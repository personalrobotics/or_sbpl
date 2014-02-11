#include <or_sbpl/TwistAction.h>
#include <math.h>

using namespace or_sbpl;


TwistAction::TwistAction() : dx(0.0), dtheta(0.0) {

    setDuration(0.0);
    setName("twist");

}

TwistAction::TwistAction(const double &dx, const double &dtheta, const double &duration) :
    dx(dx), dtheta(dtheta) {

    setDuration(duration);
    setName("twist");

}
        
/*
 * Applies the action represented by this class to the given world coordinate.
 *
 * @param wc The pose to start action propagation from
 * @param timestep The duration to apply the action
 * @return The updated pose
 */
WorldCoordinate TwistAction::apply(const WorldCoordinate &wc, const double &timestep) {

    WorldCoordinate retCoord(wc);
    retCoord.theta += dtheta*timestep;
    retCoord.x += dx*timestep*cos(retCoord.theta);
    retCoord.y += dx*timestep*sin(retCoord.theta);

    return retCoord;

}
