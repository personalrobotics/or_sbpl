#ifndef SBPL_BASE_PLANNER_ENVIRONMENT_H_
#define SBPL_BASE_PLANNER_ENVIRONMENT_H_

#include <or_sbpl/SBPLBasePlannerTypes.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>

#include <openrave/openrave.h>

namespace or_sbpl {

    class SBPLBasePlannerEnvironment : public DiscreteSpaceInformation {

    public:
        SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot);
        ~SBPLBasePlannerEnvironment() {}

        virtual bool Initialize(const double &cellsize,
                                const EnvironmentExtents &extents,
                                const int &numangles,
                                const std::vector<ActionPtr> &actions);

        virtual bool InitializeEnv(const char* sEnvFile);
        virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);

        virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
        virtual int GetGoalHeuristic(int stateID);
        virtual int GetStartHeuristic(int stateID);
        virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
        virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<ActionPtr>* ActionV);
        virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
        virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);
        virtual void SetAllPreds(CMDPSTATE* state);
        virtual int SizeofCreatedEnv();
        virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
        virtual void PrintEnv_Config(FILE* fOut);

        virtual int SetStart(const double &x, const double &y, const double &theta);
        virtual int SetGoal(const double &x, const double &y, const double &theta);
        virtual void ConvertStateIDPathIntoXYThetaPath(const std::vector<int> &state_ids,
                                                       std::vector<sbpl_xy_theta_pt_t> &path);
        

        WorldCoordinate GridCoordinateToWorldCoordinate(const GridCoordinate &gcoord) const;
        GridCoordinate WorldCoordinateToGridCoordinate(const WorldCoordinate &wcoord) const;
        GridCoordinate StateIndexToGridCoordinate(unsigned int stateidx) const;
        int GridCoordinateToStateIndex(const GridCoordinate &gcoord) const;
        int CreateState(const GridCoordinate &gc);
        bool IsValidStateId(const int &state_id) const;


        static const int INVALID_INDEX = -1;
        static const int UNINITIALIZED_ID = -1;

    protected:
        std::vector<GridCoordinate> StateId2CoordTable;
        std::map<int, int> StateIndex2StateIdTable;

    private:

        OpenRAVE::Transform WorldCoordinateToTransform(const WorldCoordinate &wcoord) const;

        OpenRAVE::RobotBasePtr _robot;         
        double _cellsize;
        double _anglesize;

        int _goal;
        int _start;

        int _gridheight;
        int _gridwidth;
        int _numangles;

        std::vector<ActionPtr> _actions;
        double _timestep; // seconds per step
    };

    typedef boost::shared_ptr<SBPLBasePlannerEnvironment> SBPLBasePlannerEnvironmentPtr;

}

#endif
