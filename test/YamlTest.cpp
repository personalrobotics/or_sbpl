#include "or_sbpl/CachedAction.h"
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <boost/unordered_map.hpp>

int main(int argc, char** argv){
    std::ifstream in_file("scripts/primitives.yaml");
    YAML::Parser parser(in_file);
    
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    boost::unordered_map<int, std::vector<or_sbpl::CachedAction> > actions;
    doc >> actions;
    
    std::cout << actions.size() << std::endl;
    std::cout << actions[0].size() << std::endl;
    // Can only do this if ._pts is not private
    //std::cout << actions[0][0]._pts.size() << std::endl;
    //std::cout << actions[0][0]._pts[3].x << " " << actions[0][0]._pts[3].y << " " << actions[0][0]._pts[3].theta << std::endl;
}
