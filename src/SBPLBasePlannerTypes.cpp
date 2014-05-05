#include <or_sbpl/SBPLBasePlannerTypes.h>

using namespace or_sbpl;

OpenRAVE::Transform WorldCoordinate::toTransform() const {


    // Rotation
    OpenRAVE::RaveTransformMatrix<double> R;
    R.rotfrommat(OpenRAVE::RaveCos(theta), -OpenRAVE::RaveSin(theta), 0.,
                 OpenRAVE::RaveSin(theta),  OpenRAVE::RaveCos(theta), 0.,
                 0., 0., 1.);

    // Translation
    OpenRAVE::RaveVector<double> t(x, y, 0.0); //TODO: Fix the z coord

    // Now put them together
    OpenRAVE::RaveTransform<double> transform(OpenRAVE::geometry::quatFromMatrix(R), t);
    return transform;


}
