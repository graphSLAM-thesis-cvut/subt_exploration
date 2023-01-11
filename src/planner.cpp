#include "planner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
/// @cond IGNORE

// An enum of supported optimal planners, alphabetical order
enum optimalPlanner
{
    PLANNER_AITSTAR,
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};

// An enum of the supported optimization objectives, alphabetical order
enum planningObjective
{
    OBJECTIVE_PATHCLEARANCE,
    OBJECTIVE_PATHLENGTH,
    OBJECTIVE_THRESHOLDPATHLENGTH,
    OBJECTIVE_WEIGHTEDCOMBO
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

// ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

// ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si);

// ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

// ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

// ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

class ValidityChecker : public ob::StateValidityChecker
{
private:
    Eigen::MatrixXf* traversability_;
    Eigen::MatrixXi* explored_;
    float slope_th_;

public:
    ValidityChecker(ob::SpaceInformationPtr& si, Eigen::MatrixXf& traversability, Eigen::MatrixXi& explored, float slope_th = 0.2) :
        ob::StateValidityChecker(si) {
            traversability_ = &traversability;
            explored_ = &explored;
            slope_th_ = slope_th;
        }

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const override
    {
        const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        int x = int(state2D->values[0]);
        int y = int(state2D->values[1]);
        if (! isindexValid(x, y, explored_)){
            return false;
        }if (!explored_->coeff(x, y)){
            return false;
        }
        if ((traversability_->coeff(x, y) > slope_th_) || (traversability_->coeff(x, y) == -1.0) ){
            return false;
        }
        return true;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const override
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const auto* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
    }
};

ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        case PLANNER_AITSTAR:
        {
            return std::make_shared<og::AITstar>(si);
            break;
        }
        // case PLANNER_BFMTSTAR:
        // {
        //     return std::make_shared<og::BFMT>(si);
        //     break;
        // }
        // case PLANNER_BITSTAR:
        // {
        //     return std::make_shared<og::BITstar>(si);
        //     break;
        // }
        // case PLANNER_CFOREST:
        // {
        //     return std::make_shared<og::CForest>(si);
        //     break;
        // }
        // case PLANNER_FMTSTAR:
        // {
        //     return std::make_shared<og::FMT>(si);
        //     break;
        // }
        // case PLANNER_INF_RRTSTAR:
        // {
        //     return std::make_shared<og::InformedRRTstar>(si);
        //     break;
        // }
        // case PLANNER_PRMSTAR:
        // {
        //     return std::make_shared<og::PRMstar>(si);
        //     break;
        // }
        // case PLANNER_RRTSTAR:
        // {
        //     return std::make_shared<og::RRTstar>(si);
        //     break;
        // }
        // case PLANNER_SORRTSTAR:
        // {
        //     return std::make_shared<og::SORRTstar>(si);
        //     break;
        // }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

ob::OptimizationObjectivePtr allocateObjective(const ob::SpaceInformationPtr& si, planningObjective objectiveType)
{
    switch (objectiveType)
    {
        // case OBJECTIVE_PATHCLEARANCE:
        //     return getClearanceObjective(si);
        //     break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        // case OBJECTIVE_THRESHOLDPATHLENGTH:
        //     return getThresholdPathLengthObj(si);
        //     break;
        // case OBJECTIVE_WEIGHTEDCOMBO:
        //     return getBalancedObjective1(si);
        //     break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

void plan(double runTime, optimalPlanner plannerType, planningObjective objectiveType, const Eigen::MatrixXf& traversability, const Eigen::MatrixXi& explored, int slope_th)
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // Set the bounds of space to be in [0,1].
    space->setBounds(0.0, 1.0);

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si, traversability, explored, slope_th));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);
    // std::string output_file = "out_plan.txt";

    if (solved)
    {
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        // if (!outputFile.empty())
        // {
        //     std::ofstream outFile(outputFile.c_str());
        //     std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
        //         printAsMatrix(outFile);
        //     outFile.close();
        // }
    }
    else
        std::cout << "No solution found." << std::endl;
}

bool isindexValid(int i, int j, const Eigen::MatrixXi* mat){
    return ( i >= 0 && j>=0 && i<mat->rows() && j<mat->cols() );
}