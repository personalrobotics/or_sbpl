#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <or_sbpl/YamlUtils.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <boost/unordered_map.hpp>

int main(int argc, char** argv){
    std::ifstream in_file("scripts/primitives.yaml");
    YAML::Parser parser(in_file);
    
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    double cellsize = 0.0;
    doc["cellsize"] >> cellsize;
    std::cout << "Cellsize: " << cellsize << std::endl;

    int numangles = 0;
    doc["numangles"] >> numangles;
    std::cout << "Num angles: " << numangles << std::endl;

    or_sbpl::ActionList actions;
    doc["actions"] >> actions;
    
    std::cout << "Num actions: " << actions.size() << std::endl;
    std::cout << "First action size: " << actions[0].size() << std::endl;
    // Can only do this if ._pts is not private
    //std::cout << actions[0][0]._pts.size() << std::endl;
    //std::cout << actions[0][0]._pts[3].x << " " << actions[0][0]._pts[3].y << " " << actions[0][0]._pts[3].theta << std::endl;
}
