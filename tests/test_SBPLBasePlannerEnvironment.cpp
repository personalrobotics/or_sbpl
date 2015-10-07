#include <gtest/gtest.h>
#include <or_sbpl/SBPLBasePlannerEnvironment.h>
#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <or_sbpl/TwistAction.h>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <openrave/openrave.h>
#include <openrave-core.h>

class SBPLBasePlannerEnvironmentTest : public testing::Test {

public:

    SBPLBasePlannerEnvironmentTest() {
        // You can do set-up work for each test here
        
        // Define six actions - forward, back, left, right, turn left, turn right
        const double half_pi = 0.5*boost::math::constants::pi<double>();

        or_sbpl::ActionPtr straight = boost::make_shared<or_sbpl::TwistAction>(1., 0., 0., 1., 1.);
        or_sbpl::ActionPtr backward = boost::make_shared<or_sbpl::TwistAction>(-1., 0., 0., 1., 1.);
        or_sbpl::ActionPtr left = boost::make_shared<or_sbpl::TwistAction>(0., 1., 0., 1., 1.);
        or_sbpl::ActionPtr right = boost::make_shared<or_sbpl::TwistAction>(0., -1., 0., 1., 1.);
        or_sbpl::ActionPtr turn_left = boost::make_shared<or_sbpl::TwistAction>(0., 0., half_pi, 1., 1.);
        or_sbpl::ActionPtr turn_right = boost::make_shared<or_sbpl::TwistAction>(0., 0., -half_pi, 1., 1.);
        std::vector<or_sbpl::ActionPtr> actions;
        actions.push_back(straight);
        actions.push_back(backward);
        actions.push_back(left);
        actions.push_back(right);
        actions.push_back(turn_left);
        actions.push_back(turn_right);

        // Environment parameters
        const double cellsize = 1.0;
        const int numangles = 4;
        const double lweight = 1.0;
        const double aweight = 2.0;
        or_sbpl::EnvironmentExtents extents(-5.0, 5.0, -2.0, 3.0);
        or_sbpl::ActionList alist;
        for(unsigned int idx=0; idx < numangles; idx++){
            alist[idx] = actions;
        }


        // Now create an openrave world and environment
        OpenRAVE::RaveInitialize(true);
        _penv = OpenRAVE::RaveCreateEnvironment();
        _robot = _penv->ReadRobotXMLFile("robots/pr2-beta-sim.robot.xml");
        _robot->SetName("PR2");
        _penv->Add(_robot);
        
        // Finally initialize the environment
        _planner_env = boost::make_shared<or_sbpl::SBPLBasePlannerEnvironment>(_robot);
        _planner_env->Initialize(cellsize,
                                 extents,
                                 numangles,
                                 alist,
                                 lweight,
                                 aweight);
    }

    virtual ~SBPLBasePlannerEnvironmentTest() {
        
        OpenRAVE::RaveDestroy();
    }

    virtual void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    // Objects declared here can be used by all tests in the test case
    or_sbpl::SBPLBasePlannerEnvironment::Ptr _planner_env;
    OpenRAVE::EnvironmentBasePtr _penv;
    OpenRAVE::RobotBasePtr _robot;

};

TEST_F(SBPLBasePlannerEnvironmentTest, GridToWorld) {

    
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
