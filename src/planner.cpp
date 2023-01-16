#include "planner.hpp"
#include "ompl/util/Console.h"
#include "ompl/control/SpaceInformation.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


const int MAP_OPEN_LIST = 1, MAP_CLOSE_LIST = 2, FRONTIER_OPEN_LIST = 3, FRONTIER_CLOSE_LIST = 4;
// const float travTh = 0.1;

const int N_S = 8;

const int offsetx[8] = {0, 0, 1, -1, 1, 1, -1, -1};
const int offsety[8] = {1, -1, 0, 0, 1, -1, 1, -1};

const int N_S4 = 4;

const int offsetx4[N_S] = {0, 0, 1, -1};
const int offsety4[N_S] = {1, -1, 0, 0};

const int MIN_FOUND = 1;
/// @cond IGNORE
// traversability - how long to enlarge obstacles
// 

// New idea:
// - During increasing boundaries, also construct a repulsive function
// - Use Djikstra algorithm, and the weight of the vertex would be max(rep(a), rep(b)) + dist(a, b)


class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
    }
};

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

// ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(lengthObj, 10.0);
    opt->addObjective(clearObj, 10.0);

    return ob::OptimizationObjectivePtr(opt);
}

// ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si);

// ob::OptimizationObjectivePtr getBalancedObjective2(const ob::SpaceInformationPtr& si);

// ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

class ValidityChecker : public ob::StateValidityChecker
{
private:
    Eigen::MatrixXf* traversability_;
    Eigen::MatrixXi* explored_;
    Eigen::MatrixXf clearity_;
    float slope_th_;
    pairs start_coords_;
    std::vector<pairs> obsts_;

public:
    ValidityChecker(ob::SpaceInformationPtr& si, Eigen::MatrixXf& traversability, Eigen::MatrixXi& explored, float slope_th, pairs start_coord) :
        ob::StateValidityChecker(si) {
            traversability_ = &traversability;
            explored_ = &explored;
            slope_th_ = slope_th;
            clearity_.resize(explored_->rows(), explored_->cols());
            start_coords_ = start_coord;

            get_obstacles();
            get_clarities();

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
        }
        // std::cout << "explored: " << explored_->coeff(x, y) << std::endl;
        if (!explored_->coeff(x, y)){
            return false;
        }
        // std::cout << "trav: " << traversability_->coeff(x, y) << " th: " << slope_th_ << std::endl;
        if ((traversability_->coeff(x, y) > slope_th_) || (traversability_->coeff(x, y) == -1.0) ){
            // std::cout << (traversability_->coeff(x, y) > slope_th_) << " " << (traversability_->coeff(x, y) == -1.0) << std::endl; 

            // std::cout << "Returning false due to traversability " << std::endl;
            return false;
        }
        // std::cout << "Returning true " << std::endl;
        return true;
    }

    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();
 
        // Extract the robot's (x,y) position from its state
        int x = int(state2D->values[0]);
        int y = int(state2D->values[1]);
 
        // Distance formula between two points, offset by the circle's
        // radius
        return clearity_(x, y);
    }

    bool get_obstacles(){
        //detect all obstacles
        std::vector<pairs> obsts_;
        auto check_cell = start_coords_;
        clearity_(check_cell.first, check_cell.second) = 0;
        // get all the obstacles cells
        std::map<pairs, int> cell_states;
        std::queue<pairs> q_m;	
        q_m.push(check_cell);
        cell_states[check_cell] = MAP_OPEN_LIST;
        int adj_vector[N_S4];
        int v_neighbours[N_S4];
        //
        // ROS_INFO("wfd 1");
        while(!q_m.empty()) {
            auto& current_cell = q_m.front();
                q_m.pop();
            if(cell_states[current_cell] == MAP_CLOSE_LIST)
                    continue;    

            int current_i = current_cell.first;
            int current_j = current_cell.second;
            for (int t = 0; t < N_S4; t++)
            {
                int i = current_i + offsetx4[t];
                int j = current_j + offsety4[t];
                auto ij = pairs(i, j);
                if (!isindexValid(i, j, explored_))
                    continue;
                if(cell_states[ij] == MAP_OPEN_LIST || cell_states[ij] == MAP_CLOSE_LIST) // ignore used cells
                    continue;
                if(explored_->coeff(i, j) && traversability_->coeff(i, j) >=0 && traversability_->coeff(i, j) <= slope_th_){ // propagate through clear cells
                    clearity_(i, j) = 0;
                    q_m.push(ij);
                    cell_states[ij] = MAP_OPEN_LIST;
                }
                if(explored_->coeff(i, j) && traversability_->coeff(i, j) >= slope_th_){ // add obstacles
                    obsts_.push_back(ij);
                    cell_states[ij] = MAP_CLOSE_LIST;
                    clearity_(i, j) = 0;
                }
            }
            cell_states[current_cell] = MAP_CLOSE_LIST;
        }

        // q_m.
        return true;//pairs(-1, -1);    
    }

     bool get_clarities(){ // call only after you call get_obstacles
        //detect all obstacles
        // std::vector<pairs> obsts_;
        // auto check_cell = start_coords_;
        // clearity_(check_cell.first, check_cell.second) = 0;
        // get all the obstacles cells
        std::map<pairs, int> cell_states;
        // std::map<pairs, int> cell_clarity;
        std::queue<pairs> q_m;	
        // q_m.push(check_cell);
        // cell_states[check_cell] = MAP_OPEN_LIST;
        for(auto& ob: obsts_){
            cell_states[ob] = MAP_OPEN_LIST;
            clearity_(ob.first, ob.second) = 0;
            q_m.push(ob);
        }
        int adj_vector[N_S4];
        int v_neighbours[N_S4];
        //
        // ROS_INFO("wfd 1");
        while(!q_m.empty()) {
            auto& current_cell = q_m.front();
                q_m.pop();
            if(cell_states[current_cell] == MAP_CLOSE_LIST)
                    continue;    

            int current_i = current_cell.first;
            int current_j = current_cell.second;
            for (int t = 0; t < N_S4; t++)
            {
                int i = current_i + offsetx4[t];
                int j = current_j + offsety4[t];
                auto ij = pairs(i, j);
                if (!isindexValid(i, j, explored_))
                    continue;
                if(cell_states[ij] == MAP_OPEN_LIST || cell_states[ij] == MAP_CLOSE_LIST) // ignore used cells
                    continue;
                if(explored_->coeff(i, j) && traversability_->coeff(i, j) >=0 && traversability_->coeff(i, j) <= slope_th_){ // propagate through clear cells, and add clarity there
                    clearity_(i, j) = 0;
                    q_m.push(ij);
                    cell_states[ij] = MAP_OPEN_LIST;
                    clearity_(i, j) = clearity_(current_i, current_j) + 1;
                }
            }
            cell_states[current_cell] = MAP_CLOSE_LIST;
        }

        // q_m.
        return true;//pairs(-1, -1);    
    }





};



ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr si, optimalPlanner plannerType)
{
    switch (plannerType)
    {
        // case PLANNER_AITSTAR:
        // {
        //     return std::make_shared<og::AITstar>(si);
        //     break;
        // }
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
        case PLANNER_RRTSTAR:
        {
            
            return std::make_shared<og::RRTstar>(si);
            break;
        }
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
        case OBJECTIVE_PATHCLEARANCE:
            return getClearanceObjective(si);
            break;
        case OBJECTIVE_PATHLENGTH:
            return getPathLengthObjective(si);
            break;
        // case OBJECTIVE_THRESHOLDPATHLENGTH:
        //     return getThresholdPathLengthObj(si);
        //     break;
        case OBJECTIVE_WEIGHTEDCOMBO:
            return getBalancedObjective(si);
            break;
        default:
            OMPL_ERROR("Optimization-objective enum is not implemented in allocation function.");
            return ob::OptimizationObjectivePtr();
            break;
    }
}

std::vector<std::pair<int, int>> plan(std::pair<int, int> startCoord, std::pair<int, int> goalCoord, 
            double runTime, optimalPlanner plannerType, planningObjective objectiveType, 
            Eigen::MatrixXf& traversability, Eigen::MatrixXi& explored, float slope_th, int upsampling_distance)
{
    std::vector<std::pair<int, int>> result;
    // Construct the robot state space in which we're planning. We're
    // planning in [0,5000]x[0,5000], a subset of R^2.
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    space->setLongestValidSegmentFraction(0.5);
    // space->setMaxi = 0.1
    // space->set

    // Set the bounds of space to be in [0,5000].
    int distanceX = std::abs(goalCoord.first - startCoord.first) + 2;
    int distanceY = std::abs(goalCoord.second - startCoord.second) + 2;
    int highX = std::min( std::max(goalCoord.first+distanceX/2, startCoord.first + distanceX/2), int(explored.rows()));
    int lowX = std::max( std::min(goalCoord.first-distanceX/2, startCoord.first - distanceX/2), 0);
    int highY = std::min( std::max(goalCoord.second+distanceY/2, startCoord.second + distanceY/2), int(explored.cols()));
    int lowY = std::max( std::min(goalCoord.second-distanceY/2, startCoord.second - distanceY/2), 0);
    ob::RealVectorBounds bounds(2);
    bounds.setHigh(0, highX);
    bounds.setLow(0, lowX);
    bounds.setHigh(1, highY);
    bounds.setLow(1, lowY);
    space->setBounds(bounds);

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));


    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(std::make_shared<ValidityChecker>(si, traversability, explored, slope_th, startCoord));
    si->setStateValidityCheckingResolution(0.5);

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = double(startCoord.first);
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = double(startCoord.second);

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = double(goalCoord.first);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = double(goalCoord.second);

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    // ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);
    og::RRTstar* RRTPlanner = new og::RRTstar(si);
    RRTPlanner->setRange(5.0);

    ob::PlannerPtr optimizingPlanner(RRTPlanner);
    
    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(0.1);
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
         og::PathGeometric path( dynamic_cast< const og::PathGeometric& >( *pdef->getSolutionPath()));
       const std::vector< ob::State* > &states = path.getStates();
       ob::State *state;
       for( size_t i = 0 ; i < states.size( ) ; ++i )
       {
            state = states[ i ]->as< ob::State >( );

            std::pair<int, int> tempCoord;
            tempCoord.first  = int(state->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            tempCoord.second = int(state->as<ob::RealVectorStateSpace::StateType>()->values[1]);

        std::cout << "calling" << std::endl;
            add_vertex_upsample(result, tempCoord, upsampling_distance);
            // result.push_back(tempCoord);
       }
    }
    else
        std::cout << "No solution found." << std::endl;
    return result;
}


void add_vertex_upsample(std::vector<std::pair<int, int>>& result, std::pair<int, int> end, int step){
    
    if(result.size() == 0){
        result.push_back(end);
        return;
    }
        // std::cout << "starting" << std::endl;
    std::pair<int, int> start = result[result.size()-1];

        // std::cout << "starting1" << std::endl;

    std::pair<float, float> prev(start.first, start.second);
        // std::cout << "starting2" << std::endl;
    
    float diffX = end.first - start.first;
    float diffY = end.second - start.second;
    float length = std::sqrt(diffX*diffX + diffY*diffY);
    float lengthPassed = step;

        // std::cout << "starting3" << std::endl;
    std::pair<float, float> step_vector(diffX/step, diffY/step);

    while (lengthPassed < length){
        std::cout << "adding " << lengthPassed << " " << length<< std::endl;
        std::pair<float, float> new_coord(prev.first+step_vector.first, prev.second+step_vector.second);
        result.push_back(std::pair<int, int>(new_coord.first, new_coord.second));
        prev = new_coord;
        lengthPassed += step;
    }
    result.push_back(end);
}



bool isindexValid(int i, int j, const Eigen::MatrixXi* mat){
    return ( i >= 0 && j>=0 && i<mat->rows() && j<mat->cols() );
}