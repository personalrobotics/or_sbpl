#ifndef SBPL_BASE_PLANNER_TYPES_H_
#define SBPL_BASE_PLANNER_TYPES_H_

#include <iostream>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>

#include <openrave/openrave.h>

namespace or_sbpl {

    class WorldCoordinate {
    public:
    WorldCoordinate() : x(0.0), y(0.0), theta(0.0) {};
    WorldCoordinate(const double &x, const double &y, const double &theta) : x(x), y(y), theta(theta) {};
    WorldCoordinate(const WorldCoordinate &wc) : x(wc.x), y(wc.y), theta(wc.theta) {}

        std::string toString() { return (boost::format("[ %0.3f, %0.3f, %0.3f]") % x % y % theta).str(); }

        /**
         * Converts a (x,y,theta) pose to a transform for the robot
         *
         * @param wcoord The pose to convert
         * @return The associated transform
         */
        OpenRAVE::Transform toTransform() const;

        double x; 
        double y;
        double theta;

        friend std::ostream& operator<< (std::ostream &out, WorldCoordinate &wc){
            out << wc.toString();
            return out;
        }
    };
   

    class GridCoordinate {
    public: 
    GridCoordinate() : x(0), y(0), theta(0) {};
    GridCoordinate(const int &x, const int &y, const int &theta) : x(x), y(y), theta(theta) {}
    GridCoordinate(const GridCoordinate &gc) : x(gc.x), y(gc.y), theta(gc.theta) {}

        std::string toString() { return (boost::format("[ %d, %d, %d]") % x % y % theta).str(); }

        int x;
        int y;
        int theta;

        friend std::ostream& operator<< (std::ostream &out, GridCoordinate &gc){
            out << gc.toString();
            return out;
        }
    };

    class Action {
    public:
        virtual bool apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const = 0;
        virtual std::vector<WorldCoordinate> applyWithIntermediates(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot) const = 0;

        std::string getName() const { return _name; }
        void setName(const std::string &name) { _name = name; }

    protected:
        std::string _name;
    };

    typedef boost::shared_ptr<Action> ActionPtr;


    class EnvironmentExtents {
    public:
    EnvironmentExtents() : xmin(0.0), xmax(0.0), ymin(0.0), ymax(0.0) {}
    EnvironmentExtents(const double &xmin, const double &xmax, const double &ymin, const double &ymax) :
        xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {};

        double xmin;
        double xmax;
        double ymin;
        double ymax;
    };

    class PlannedWaypoint {
    public:
    PlannedWaypoint(const WorldCoordinate &coord, const ActionPtr& action) : coord(coord), action(action) {}
	WorldCoordinate coord;
	ActionPtr action;
    };

    typedef boost::shared_ptr<PlannedWaypoint> PlannedWaypointPtr;
    
}

#endif
