#include <gtest/gtest.h>
#include <or_sbpl/SBPLBasePlanner.h>
#include <boost/make_shared.hpp>
#include <openrave/openrave.h>
#include <openrave/planner.h>
#include <openrave-core.h>
#include <yaml-cpp/yaml.h>
#include "OpenRAVETestEnvironment.h"

class SBPLBasePlannerTest : public testing::Test {
public:
    void SetUp() {
        OpenRAVE::PlannerBase::PlannerParametersPtr params =
            boost::make_shared<OpenRAVE::PlannerBase::PlannerParameters>();

        _lweight = 2.0;
        _aweight = 4.0;
        _cellsize = 1.0;
        _numangles = 4;
        _timelimit = 60.0;

        // Now initialize some yaml with all the important parameters
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "linear_weight" << YAML::Value << _lweight;
        emitter << YAML::Key << "theta_weight" << YAML::Value << _aweight;
        emitter << YAML::Key << "cellsize" << YAML::Value << _cellsize;
        emitter << YAML::Key << "numangles" << YAML::Value << _numangles;
        emitter << YAML::Key << "timelimit" << YAML::Value << _timelimit;
        emitter << YAML::Key << "extents" << YAML::Value;
        emitter << YAML::BeginSeq << -5.0 << 5.0 << -2.0 << 3.0 << YAML::EndSeq;

        emitter << YAML::Key << "actions" << YAML::Value;
        emitter << YAML::BeginSeq;

        // Angle 0
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "angle" << YAML::Value << 0;
        emitter << YAML::Key << "poses" << YAML::Value;
        emitter << YAML::BeginSeq;
        // forward
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 0.0 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 1.0 << 0.0  << 0.0 << YAML::EndSeq;
        emitter << YAML::EndSeq;
        // left
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 0.0 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 1.57079 << YAML::EndSeq;
        emitter << YAML::EndSeq;
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;

        // Angle 1
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "angle" << YAML::Value << 1;
        emitter << YAML::Key << "poses" << YAML::Value;
        emitter << YAML::BeginSeq;
        // forward
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 1.57079 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 0.0 << 1.0  << 1.57079 << YAML::EndSeq;
        emitter << YAML::EndSeq;
        // left
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 1.57079 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 3.14159 << YAML::EndSeq;
        emitter << YAML::EndSeq; 
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;

        // Angle 2
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "angle" << YAML::Value << 1;
        emitter << YAML::Key << "poses" << YAML::Value;
        emitter << YAML::BeginSeq;
        // forward
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 3.14159 << YAML::EndSeq;
        emitter << YAML::BeginSeq << -1.0 << 0.0  << 3.14159 << YAML::EndSeq;
        emitter << YAML::EndSeq;
        // left
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 3.14159 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 4.71238 << YAML::EndSeq;
        emitter << YAML::EndSeq; 
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;

        // Angle 3
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "angle" << YAML::Value << 1;
        emitter << YAML::Key << "poses" << YAML::Value;
        emitter << YAML::BeginSeq;
        // forward
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 4.71238 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 0.0 << -1.0  << 4.71238 << YAML::EndSeq;
        emitter << YAML::EndSeq;
        // left
        emitter << YAML::BeginSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 4.71238 << YAML::EndSeq;
        emitter << YAML::BeginSeq << 0.0 << 0.0  << 0.0 << YAML::EndSeq;
        emitter << YAML::EndSeq; 
        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;

        emitter << YAML::EndSeq;
        emitter << YAML::EndMap;

        params->_sExtraParameters = emitter.c_str();


        // Now create an openrave world and environment
        _penv = OpenRAVE::RaveCreateEnvironment();
        _robot = _penv->ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml");
        _robot->SetName("PR2");
        _penv->Add(_robot);
        
        // Finally initialize the environment
        _planner = boost::make_shared<or_sbpl::SBPLBasePlanner>(_penv);
        _planner->InitPlan(_robot, params);

    }


    double _lweight;
    double _aweight;
    double _cellsize;
    int _numangles;
    double _timelimit;

    or_sbpl::SBPLBasePlannerPtr _planner;
    OpenRAVE::EnvironmentBasePtr _penv;
    OpenRAVE::RobotBasePtr _robot;

};

TEST_F(SBPLBasePlannerTest, TestParameterParsing) {

    or_sbpl::SBPLBasePlannerEnvironmentConstPtr planner_env = _planner->GetPlannerEnv();
    double allowed_err = 1e-8;
    ASSERT_NEAR(planner_env->GetCellsize(), _cellsize, allowed_err);
    ASSERT_EQ(planner_env->GetNumAngles(), _numangles);
    ASSERT_NEAR(planner_env->GetLinearWeight(), _lweight, allowed_err);
    ASSERT_NEAR(planner_env->GetAngularWeight(), _aweight, allowed_err);
    
    or_sbpl::EnvironmentExtents extents = planner_env->GetExtents();
    ASSERT_NEAR(extents.xmin, -5.0, allowed_err);
    ASSERT_NEAR(extents.xmax, 5.0, allowed_err);
    ASSERT_NEAR(extents.ymin, -2.0, allowed_err);
    ASSERT_NEAR(extents.ymax, 3.0, allowed_err);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    testing::AddGlobalTestEnvironment(new OpenRAVETestEnvironment);
    return RUN_ALL_TESTS();
}
