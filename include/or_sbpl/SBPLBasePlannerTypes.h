#ifndef SBPL_BASE_PLANNER_TYPES_H_
#define SBPL_BASE_PLANNER_TYPES_H_

#include <iostream>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <openrave/openrave.h>

namespace or_sbpl {

    /**
     * Describes a coordinate in world frame
     */
    class WorldCoordinate {
    public:
	/**
	 * Constructor.  The coordinate values are defaulted to zero.
	 */
    WorldCoordinate() : x(0.0), y(0.0), theta(0.0) {};

	/**
	 * Constructor
	 *
	 * @param x The x value of the coordinate
	 * @param y The y value of the coordinate
	 * @param theta The theta value of the coordinate
	 */
    WorldCoordinate(const double &x, const double &y, const double &theta) : x(x), y(y), theta(theta) {};
	
	/**
	 * Copy constructor
	 *
	 * @param wc The coordinate to copy
	 */
    WorldCoordinate(const WorldCoordinate &wc) : x(wc.x), y(wc.y), theta(wc.theta) {}

	/**
	 * @return The string representation of the coordinate
	 */
        std::string toString() const { return (boost::format("[ %0.3f, %0.3f, %0.3f]") % x % y % theta).str(); }

        /**
         * Converts this world coordinate to a transform for the robot
         *
         * @return The associated transform
         */
        OpenRAVE::Transform toTransform() const;

	/**
	 * The x value of the coordinate
	 */
        double x; 

	/**
	 * The y value of the coordinate
	 */
        double y;

	/**
	 * The theta value of the coordinate
	 */
        double theta;

	/**
	 * Output stream overload
	 *
	 * @param out The output stream
	 * @param wc The world coordinate to write to the stream
	 */
        friend std::ostream& operator<< (std::ostream &out, WorldCoordinate &wc){
            out << wc.toString();
            return out;
        }
    };

    /**
     * Describes a coordinate in grid space
     */
    class GridCoordinate {
    public:

	/**
	 * Constructor. The coordinate values are defaulted to zero
	 */
    GridCoordinate() : x(0), y(0), theta(0) {};
	
	/**
	 * Constructor.
	 *
	 * @param x The x value of the coordinate
	 * @param y The y value of the coordinate
	 * @param theta The theta value of the coordinate
	 */
    GridCoordinate(const int &x, const int &y, const int &theta) : x(x), y(y), theta(theta) {}

	/**
	 * Copy constructor.
	 *
	 * @param gc The coordinate to copy
	 */
    GridCoordinate(const GridCoordinate &gc) : x(gc.x), y(gc.y), theta(gc.theta) {}

	/**
	 * @return The string representation of the coordinate
	 */
        std::string toString() const { return (boost::format("[ %d, %d, %d]") % x % y % theta).str(); }

	/**
	 * The x value of the coordinate
	 */
        int x;
	
	/**
	 * The y value of the coordinate
	 */
        int y;

	/**
	 * The theta value of the coordinate
	 */
        int theta;

	/**
	 * Output stream overload
	 *
	 * @param out The output stream
	 * @param gc The grid coordinate to write to the stream
	 */
        friend std::ostream& operator<< (std::ostream &out, GridCoordinate &gc){
            out << gc.toString();
            return out;
        }
    };

    /**
     * A class to represent an action for the lattice
     */
    class Action {
    public:
	/**
	 * Apply this action
	 *
	 * @param wc The coordinate to begin applying the action from
	 * @param robot The robot to use for collision checking
	 * @param final_wc The final coordinate after the full action is applied (set by this function)
	 * @return True if the action was successfully applied
	 */
        virtual bool apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const = 0;
	
	/**
	 * Apply this action
	 *
	 * @param wc The coordinate to begin applying the action from
	 * @param robot The robot to use for collision checking
	 * @return A list of all intermediate poses visited by this action
	 */
        virtual std::vector<WorldCoordinate> applyWithIntermediates(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot) const = 0;

	/**
	 * @return The name for this action
	 */
        std::string getName() const { return _name; }
	
	/**
	 * @param name The name for this action
	 */
        void setName(const std::string &name) { _name = name; }

	/**
	 * @return The weight to apply to the cost of this action
	 */
	double getWeight() const { return _weight; }

	/**
	 * @param weight The weight to apply to the cost of this action
	 */
	void setWeight(const double &weight) { _weight = weight; }

    protected:
	/**
	 * The name of this action
	 */
        std::string _name;
	
	/**
	 * The weight to apply to the cost of this action
	 */
	double _weight;
    };

    typedef boost::shared_ptr<Action> ActionPtr;


    /**
     * A class to represent the bounds of the search space in world frame
     */
    class EnvironmentExtents {
    public:
	/**
	 * Constructor. All bounds are defaulted to zero
	 */
    EnvironmentExtents() : xmin(0.0), xmax(0.0), ymin(0.0), ymax(0.0) {}

	/**
	 * Constructor
	 *
	 * @param xmin The minimum x value
	 * @param xmax The maximum x value
	 * @param ymin The minimum y value
	 * @param ymax The maximum y value
	 */
    EnvironmentExtents(const double &xmin, const double &xmax, const double &ymin, const double &ymax) :
        xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {};

	/**
	 * The minimum x value
	 */
        double xmin;

	/**
	 * The maximum x value
	 */
        double xmax;
	/**
	 * The minimum y value
	 */
        double ymin;

	/**
	 * The maximum y value
	 */
        double ymax;
    };

    /**
     * A waypoint that pairs a start point and an action
     */
    class PlannedWaypoint {
    public:
	/**
	 * Constructor
	 *
	 * @param coord The coordinate of this waypoint
	 * @param action The action to apply to get to this waypoint
	 */
    PlannedWaypoint(const WorldCoordinate &coord, const ActionPtr& action) : coord(coord), action(action) {}

	/**
	 * The coordinate for this waypoint
	 */
	WorldCoordinate coord;

	/**
	 * The action to apply to get to this waypoint
	 */
	ActionPtr action;
    };

    typedef boost::shared_ptr<PlannedWaypoint> PlannedWaypointPtr;

    // Map orientation grid coordinate to list of available actions
    typedef std::map<unsigned int, std::vector<ActionPtr> > ActionList;


    
}

#endif
