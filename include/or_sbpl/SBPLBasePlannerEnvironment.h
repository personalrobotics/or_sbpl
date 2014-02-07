#ifndef SBPL_BASE_PLANNER_ENVIRONMENT_H_
#define SBPL_BASE_PLANNER_ENVIRONMENT_H_

#include <sbpl/discrete_space_information/environment.h>

#include <openrave/openrave.h>

namespace or_sbpl {

    struct WorldCoordinate {
        double x; 
        double y;
        double theta;
    };

    struct GridCoordinate {
        int x;
        int y;
        int theta;
    };

    class SBPLBasePlannerEnvironment : public DiscreteSpaceInformation {
    public:
        SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot);
        ~SBPLBasePlannerEnvironment() {}

        virtual bool InitializeEnv(const char* sEnvFile);
        virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);

        virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
        virtual int GetGoalHeuristic(int stateID);
        virtual int GetStartHeurisitic(int stateID);
        virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
        virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
        virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);
        virtual void SetAllPreds(CMDPSTATE* state);
        virtual int SizeofCreatedEnv();
        virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
        virtual void PrintEnv_Config(FILE* fOut);

        WorldCoordinate GridCoordinateToWorldCoordinate(const GridCoordinate &gcoord) const;
        GridCoordinate WorldCoordinateToGridCoordinate(const WorldCoordinate &wcoord) const;
        GridCoordinate StateIndexToGridCoordinate(unsigned int stateidx) const;

    protected:
        std::vector<GridCoordinate*> StateId2CoordTable;
        std::vector<int> StateIndex2StateIdTable;

    private:
        OpenRAVE::RobotBasePtr _robot;         
        double _cellsize;
        double _anglesize;

        int _goal;
        int _start;
    };

    typedef boost::shared_ptr<SBPLBasePlannerEnvironment> SBPLBasePlannerEnvironmentPtr;

}

#endif
