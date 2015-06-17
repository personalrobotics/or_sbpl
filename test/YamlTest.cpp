#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <or_sbpl/YamlUtils.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

int main(int argc, char** argv){

#ifdef YAMLCPP_NEWAPI
    YAML::Node doc = YAML::LoadFile("scripts/primitives.yaml");
#else
    std::ifstream in_file("scripts/primitives.yaml");
    YAML::Parser parser(in_file);
    
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif
    
    double cellsize = 0.0;
#ifdef YAMLCPP_NEWAPI
    cellsize = doc["cellsize"].as<double>();
#else
    doc["cellsize"] >> cellsize;
#endif
    std::cout << "Cellsize: " << cellsize << std::endl;

    int numangles = 0;
#ifdef YAMLCPP_NEWAPI
    numangles = doc["numangles"].as<int>();
#else
    doc["numangles"] >> numangles;
#endif
    std::cout << "Num angles: " << numangles << std::endl;

    
    double linear_weight = 0.0;
#ifdef YAMLCPP_NEWAPI
    linear_weight = doc["linear_weight"].as<double>();
#else
    doc["linear_weight"] >> linear_weight;
#endif
    std::cout << "Linear weight: " << linear_weight << std::endl;

    double theta_weight = 0.0;
#ifdef YAMLCPP_NEWAPI
    theta_weight = doc["theta_weight"].as<double>();
#else
    doc["theta_weight"] >> theta_weight;
#endif
    std::cout << "Theta weight: " << theta_weight << std::endl;

    or_sbpl::ActionList actions;
    doc["actions"] >> actions;
    std::cout << "Num actions: " << actions.size() << std::endl;

    BOOST_FOREACH(or_sbpl::ActionList::value_type &alist, actions){
	BOOST_FOREACH(or_sbpl::ActionPtr a, alist.second){
	    or_sbpl::CachedActionPtr ca = boost::dynamic_pointer_cast<or_sbpl::CachedAction>(a);
	    std::cout << "Added action with weight: " << ca->getWeight() << std::endl;
	    std::cout << "Poses: " << std::endl;
	    std::vector<or_sbpl::WorldCoordinate> pts = ca->getPoints();
	    for(unsigned int i=0; i < pts.size(); i++){
		or_sbpl::WorldCoordinate wc = pts[i];
		std::cout << "\t" << wc << std::endl;
	    }
	}
    }
}
