#ifndef SBPL_BASE_PLANNER_TYPES_H_
#define SBPL_BASE_PLANNER_TYPES_H_

#include <iostream>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>

namespace or_sbpl {

    class WorldCoordinate {
    public:
    WorldCoordinate() : x(0.0), y(0.0), theta(0.0) {};
    WorldCoordinate(const double &x, const double &y, const double &theta) : x(x), y(y), theta(theta) {};
    WorldCoordinate(const WorldCoordinate &wc) : x(wc.x), y(wc.y), theta(wc.theta) {}

        std::string toString() { return (boost::format("[ %0.3f, %0.3f, %0.3f]") % x % y % theta).str(); }
        
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
    Action() : dx(0.0), dy(0.0), dtheta(0.0), duration(0.0) {};
    Action(const double &dx, const double &dy, const double &dtheta, const double &duration) :
        dx(dx), dy(dy), dtheta(dtheta), duration(duration) {};
        
        virtual WorldCoordinate apply(const WorldCoordinate &wc, const double &duration) const = 0;

        double dx;
        double dy;
        double dtheta;
        double duration;
    };

    typedef boost::shared_ptr<Action> ActionPtr;

    
}

#endif
