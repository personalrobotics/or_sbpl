
#include <or_sbpl/TwistAction.h>
#include <math.h>

#include <boost/math/constants/constants.hpp>

using namespace or_sbpl;
namespace bmc = boost::math::constants;

TwistAction::TwistAction() : dx(0.0), dy(0.0), dtheta(0.0) {

    setDuration(0.0);
    setName("twist");

}

TwistAction::TwistAction(const double &dx, const double &dy, const double &dtheta, const double &duration) :
    dx(dx), dy(dy), dtheta(dtheta) {

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
WorldCoordinate TwistAction::apply(const WorldCoordinate &wc, const double &timestep) const {

    WorldCoordinate retCoord(wc);
    retCoord.theta += dtheta*timestep;
    retCoord.x += dx*timestep*cos(retCoord.theta) + dy*timestep*cos(retCoord.theta + 0.5*bmc::pi<double>());
    retCoord.y += dx*timestep*sin(retCoord.theta) + dy*timestep*sin(retCoord.theta + 0.5*bmc::pi<double>());

    return retCoord;

}
