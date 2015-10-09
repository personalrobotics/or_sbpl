#include <gtest/gtest.h>
#include <or_sbpl/SBPLBasePlannerEnvironment.h>
#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <or_sbpl/TwistAction.h>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <openrave/openrave.h>
#include <openrave-core.h>
#include "OpenRAVETestEnvironment.h"

class SBPLBasePlannerEnvironmentTest : public testing::Test {

public:

    void SetUp() {
        // Code here will be called immediately after the constructor (right
        // before each test).
        
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
        _lweight = 1.0;
        _aweight = 2.0;
        or_sbpl::EnvironmentExtents extents(-5.0, 5.0, -2.0, 3.0);
        or_sbpl::ActionList alist;
        for(unsigned int idx=0; idx < numangles; idx++){
            alist[idx] = actions;
        }


        // Now create an openrave world and environment
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
                                 _lweight,
                                 _aweight);

        // Create all the states in the world
        for(int gx=0; gx < 10; gx++){
            for(int gy=0; gy < 5; gy++){
                for(int gt=0; gt < 4; gt++){
                    or_sbpl::GridCoordinate gc(gx, gy, gt);
                    _planner_env->CreateState(gc);
                }
            }
        }

        // Start and goal
        _start.x = -3.8;
        _start.y = -1.4;
        _start.theta = 0.0;

        _goal.x = 3.4;
        _goal.y = 2.6;
        _goal.theta = 5.0;
    }

    virtual void TearDown() {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    void _GridToWorld(const int &gx, const int &gy, const int &gt,
                      const double &wx, const double &wy, const double &wt) {

        double allowed_err = 1e-4;

        or_sbpl::GridCoordinate gc(gx, gy, gt);
        or_sbpl::WorldCoordinate wc = _planner_env->GridCoordinateToWorldCoordinate(gc);
        ASSERT_NEAR(wc.x, wx, allowed_err);
        ASSERT_NEAR(wc.y, wy, allowed_err);
        ASSERT_NEAR(wc.theta, wt, allowed_err);
    }

    void _WorldToGrid(const double &wx, const double &wy, const double &wt,
                      const int &gx, const int &gy, const int &gt) {
        
        or_sbpl::WorldCoordinate wc(wx, wy, wt);
        or_sbpl::GridCoordinate gc = _planner_env->WorldCoordinateToGridCoordinate(wc);
        ASSERT_EQ(gc.x, gx);
        ASSERT_EQ(gc.y, gy);
        ASSERT_EQ(gc.theta, gt);

    }

    void _GridToState(const int &gx, const int &gy, const int &gt,
                      const unsigned int &sidx) {
        
        or_sbpl::GridCoordinate gc(gx, gy, gt);
        int state_idx = _planner_env->GridCoordinateToStateIndex(gc);

        ASSERT_EQ(state_idx, sidx);
        
    }

    void _StateToGrid(const unsigned int &sidx, 
                      const int &gx, const int &gy, const int &gt) {

        or_sbpl::GridCoordinate gc = _planner_env->StateIndexToGridCoordinate(sidx);

        ASSERT_EQ(gc.x, gx);
        ASSERT_EQ(gc.y, gy);
        ASSERT_EQ(gc.theta, gt);
    }

    void _ComputeHeuristic(const double &sx, const double &sy, const double &st,
                           const double &ex, const double &ey, const double &et,
                           const int &computed_h) {
        
        double pi = boost::math::constants::pi<double>();

        // Angle diff - angles between 0 and 2pi
        double d1 = et - st;
        if(d1 < 0.0) d1 += 2.*pi;
        
        double d2 = st - et;
        if(d2 < 0.0) d2 += 2.*pi;

        double h = _lweight*1000.*std::sqrt((ex - sx)*(ex - sx) + (ey - sy)*(ey - sy)) +
            _aweight*(std::min(d1, d2));
        int expected_h = int(h);
        ASSERT_EQ(computed_h, expected_h);
    }

    void _GoalHeuristic(const double &sx, const double &sy, const double &st,
                        const unsigned int &sidx) {

        double pi = boost::math::constants::pi<double>();
        or_sbpl::WorldCoordinate goal_coord(3.5, 2.5, 1.5*pi);

        int ph = _planner_env->GetGoalHeuristic(sidx);
        _ComputeHeuristic(sx, sy, st, 
                          goal_coord.x, goal_coord.y, goal_coord.theta,
                          ph);
    }

    void _StartHeuristic(const double &sx, const double &sy, const double &st,
                         const unsigned int &sidx) {

        double pi = boost::math::constants::pi<double>();
        or_sbpl::WorldCoordinate start_coord(-3.5, -1.5, 0.);

        int ph = _planner_env->GetStartHeuristic(sidx);
        _ComputeHeuristic(sx, sy, st, 
                          start_coord.x, start_coord.y, start_coord.theta,
                          ph);
    }

    template<typename T>
    void _CheckVectors(const std::vector<T> expected,
                       const std::vector<T> actual) {
        ASSERT_EQ(actual.size(), expected.size());
        for(unsigned int idx=0; idx < actual.size(); idx++){
            ASSERT_EQ(actual[idx], expected[idx]);
        }
    }

    void _CheckWaypoint(const or_sbpl::PlannedWaypointPtr &wpt,
                        const or_sbpl::WorldCoordinate &wc) {
        ASSERT_EQ(wpt->coord.x, wc.x);
        ASSERT_EQ(wpt->coord.y, wc.y);
        ASSERT_EQ(wpt->coord.theta, wc.theta);
    }
    

    // Objects declared here can be used by all tests in the test case
    or_sbpl::SBPLBasePlannerEnvironmentPtr _planner_env;
    OpenRAVE::EnvironmentBasePtr _penv;
    OpenRAVE::RobotBasePtr _robot;

    or_sbpl::WorldCoordinate _start;
    or_sbpl::WorldCoordinate _goal;
    
    double _lweight;
    double _aweight;
};

TEST_F(SBPLBasePlannerEnvironmentTest, SizeOfEnv) {
    ASSERT_EQ(_planner_env->SizeofCreatedEnv(), 200);
}

TEST_F(SBPLBasePlannerEnvironmentTest, GridToWorld) {

    double pi = boost::math::constants::pi<double>();

    _GridToWorld(0, 2, 0, -4.5, 0.5, 0.);
    _GridToWorld(2, 3, 1, -2.5, 1.5, 0.5*pi);
    _GridToWorld(3, 0, 2, -1.5, -1.5, pi);
    _GridToWorld(4, 1, 3, -0.5, -0.5, 1.5*pi);
    _GridToWorld(5, 2, 0, 0.5, 0.5, 0.);
    _GridToWorld(6, 0, 1, 1.5, -1.5, 0.5*pi);
    _GridToWorld(7, 4, 2, 2.5, 2.5, pi);
    _GridToWorld(9, 3, 3, 4.5, 1.5, 1.5*pi);
}

TEST_F(SBPLBasePlannerEnvironmentTest, WorldToGrid) {

    double pi = boost::math::constants::pi<double>();
    _WorldToGrid(-4.1, 0.8, 0.8, 0, 2, 1);
    _WorldToGrid(-2.8, 1.1, 1.6, 2, 3, 1);
    _WorldToGrid(-1.6, -1.9, 3.5, 3, 0, 2);
    _WorldToGrid(-0.1, -0.1, 5.15, 4, 1, 3);
    _WorldToGrid(0.78, 0.5, 0.0001, 5, 2, 0);
    _WorldToGrid(1.001, -1.001, 3.14, 6, 0, 2);
    _WorldToGrid(2.999, 2.001, 3.93, 7, 4, 3);
    _WorldToGrid(4.999, 1.3, 6.27, 9, 3, 0);

}

TEST_F(SBPLBasePlannerEnvironmentTest, GridCoordinateToStateIndex) {
    
    // Grid stored in theta, y, x order

    _GridToState(0, 2, 0, 8);
    _GridToState(2, 3, 1, 53);
    _GridToState(3, 0, 2, 62);
    _GridToState(4, 1, 3, 87);
    _GridToState(5, 2, 0, 108);
    _GridToState(6, 0, 1, 121);
    _GridToState(7, 4, 2, 158);
    _GridToState(9, 3, 3, 195);

}

TEST_F(SBPLBasePlannerEnvironmentTest, StateIndexToGridCoordinate) {

    _StateToGrid(8, 0, 2, 0);
    _StateToGrid(53, 2, 3, 1);
    _StateToGrid(62, 3, 0, 2);
    _StateToGrid(87, 4, 1, 3);
    _StateToGrid(108, 5, 2, 0);
    _StateToGrid(121, 6, 0, 1);
    _StateToGrid(158, 7, 4, 2);
    _StateToGrid(195, 9, 3, 3);

}

TEST_F(SBPLBasePlannerEnvironmentTest, SetStartSetGoal) {

    int sid = _planner_env->SetStart(_start.x, _start.y, _start.theta);
    ASSERT_EQ(sid, 20);

    int gid = _planner_env->SetGoal(_goal.x, _goal.y, _goal.theta);
    ASSERT_EQ(gid, 179);    
}

TEST_F(SBPLBasePlannerEnvironmentTest, GoalHeuristicCost) {
    int gid = _planner_env->SetGoal(_goal.x, _goal.y, _goal.theta);

    double pi = boost::math::constants::pi<double>();
    _GoalHeuristic(-4.5, 0.5, 0, 8);
    _GoalHeuristic(-2.5, 1.5, 0.5*pi, 53);
    _GoalHeuristic(-1.5, -1.5, pi, 62);
    _GoalHeuristic(-0.5, -0.5, 1.5*pi, 87);
    _GoalHeuristic(0.5, 0.5, 0, 108);
    _GoalHeuristic(1.5, -1.5, 0.5*pi, 121);
    _GoalHeuristic(2.5, 2.5, pi, 158);
    _GoalHeuristic(4.5, 1.5, 1.5*pi, 195);
}

TEST_F(SBPLBasePlannerEnvironmentTest, StartHeuristicCost) {
    double pi = boost::math::constants::pi<double>();

    int sid = _planner_env->SetStart(_start.x, _start.y, _start.theta);
    _StartHeuristic(-4.5, 0.5, 0, 8);
    _StartHeuristic(-2.5, 1.5, 0.5*pi, 53);
    _StartHeuristic(-1.5, -1.5, pi, 62);
    _StartHeuristic(-0.5, -0.5, 1.5*pi, 87);
    _StartHeuristic(0.5, 0.5, 0, 108);
    _StartHeuristic(1.5, -1.5, 0.5*pi, 121);
    _StartHeuristic(2.5, 2.5, pi, 158);
    _StartHeuristic(4.5, 1.5, 1.5*pi, 195);
}

TEST_F(SBPLBasePlannerEnvironmentTest, HeuristicCost) {

    double pi = boost::math::constants::pi<double>();
    _ComputeHeuristic(-4.5, 0.5, 0., 
                      -1.5, -1.5, pi,
                      _planner_env->GetFromToHeuristic(8, 62));
    _ComputeHeuristic(4.5, 1.5, 1.5*pi,
                      -0.5, -0.5, 1.5*pi,
                      _planner_env->GetFromToHeuristic(195, 87));
    _ComputeHeuristic(-2.5, 1.5, 0.5*pi,
                      2.5, 2.5, pi,
                      _planner_env->GetFromToHeuristic(53, 158));
}

TEST_F(SBPLBasePlannerEnvironmentTest, IsValidState) {

    // Assume all states are initialized in constructor
    ASSERT_TRUE(_planner_env->IsValidStateId(0));
    ASSERT_TRUE(_planner_env->IsValidStateId(53));
    ASSERT_TRUE(_planner_env->IsValidStateId(62));
    ASSERT_TRUE(_planner_env->IsValidStateId(87));
    ASSERT_TRUE(_planner_env->IsValidStateId(108));
    ASSERT_TRUE(_planner_env->IsValidStateId(121));
    ASSERT_TRUE(_planner_env->IsValidStateId(158));
    ASSERT_TRUE(_planner_env->IsValidStateId(199));

    ASSERT_FALSE(_planner_env->IsValidStateId(-1));
    ASSERT_FALSE(_planner_env->IsValidStateId(1000));
    ASSERT_FALSE(_planner_env->IsValidStateId(200));
}

TEST_F(SBPLBasePlannerEnvironmentTest, GetSuccessors) {
    double pi = boost::math::constants::pi<double>();

    std::vector<int> sids;
    std::vector<int> scosts;
    std::vector<or_sbpl::ActionPtr> sactions;

    // First test something in the middle of the grid
    _planner_env->GetSuccs(108, &sids, &scosts, &sactions);
    ASSERT_EQ(sids.size(), 6);
    ASSERT_EQ(scosts.size(), 6);
    ASSERT_EQ(sactions.size(), 6);

    int expected_successors[] = {128, 88, 112, 104, 109, 111};
    std::vector<int> eids(expected_successors, expected_successors + sizeof(expected_successors)/sizeof(int));
    _CheckVectors(eids, sids);


    int linear_cost = int(_lweight*1000.*1.);
    int angular_cost = int(_aweight*0.5*pi);
    double expected_costs[] = {linear_cost, linear_cost, linear_cost, linear_cost, 
                               angular_cost, angular_cost};
    std::vector<int> ecosts(expected_costs, expected_costs + sizeof(expected_costs)/sizeof(double));
    _CheckVectors(ecosts, scosts);

    // Now test something on the edge
    _planner_env->GetSuccs(8, &sids, &scosts, &sactions);
    ASSERT_EQ(sids.size(), 5);
    ASSERT_EQ(scosts.size(), 5);
    ASSERT_EQ(sactions.size(), 5);

    int expected_successors2[] = {28, 12, 4, 9, 11};
    std::vector<int> eids2(expected_successors2, expected_successors2 + sizeof(expected_successors2)/sizeof(int));
    _CheckVectors(eids2, sids);

    double expected_costs2[] = {linear_cost, linear_cost, linear_cost, 
                               angular_cost, angular_cost};
    std::vector<int> ecosts2(expected_costs2, expected_costs2 + sizeof(expected_costs2)/sizeof(double));
    _CheckVectors(ecosts2, scosts);

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    testing::AddGlobalTestEnvironment(new OpenRAVETestEnvironment);
    return RUN_ALL_TESTS();
}
