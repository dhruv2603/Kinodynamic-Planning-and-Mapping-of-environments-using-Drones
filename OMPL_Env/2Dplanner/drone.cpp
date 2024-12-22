///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Dhruv, Hrishikesh, Shreyas
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>

/*Include the Base ompl files*/
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
/*Include the control ompl files*/
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/PathControl.h>
/*Include Benchmarking files*/
#include <ompl/tools/benchmark/Benchmark.h>
/*Collision Checker*/
#include "CollisionChecking.h"

/*Global Variables*/
double MOVE_DISTANCE = 4.0;
double VISIBLE_REGION = 8.0;


void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, const double duration, ompl::base::State* result)
{
    // Create an SO2 state space instance
    ompl::base::SO2StateSpace SO2;

    // Access the compound state
    ompl::base::CompoundStateSpace::StateType* s = result->as<ompl::base::CompoundStateSpace::StateType>();

    // Access the yaw SO2 subspace
    ompl::base::SO2StateSpace::StateType* yawState = s->as<ompl::base::SO2StateSpace::StateType>(1);

    // Normalize the yaw angle to ensure it stays within [-pi, pi]
    SO2.enforceBounds(yawState);
}

void droneODE(const ompl::control::ODESolver::StateType &q /* q */, const ompl::control::Control *c /* control */,
            ompl::control::ODESolver::StateType &qdot /* qdot */)
{
    // TODO: Fill in the ODE for the drone's dynamics
    // State Space: {x,y,psi,dot_x,dot_y,dot_psi}
    const double *u  = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double F   = u[0];               //Total thrust
    const double Mz = u[1];                //Torque around z-axis

    const double psi       = q[2];         //yaw
    const double dot_x     = q[3];         //vel_x
    const double dot_y     = q[4];         //vel_y
    const double dot_psi   = q[5];         //ang_yaw_vel

    // Constants
    const double m = 1.0;  // Mass of the quadrotor
    const double g = 9.81; // Acceleration due to gravity
    const double I_z = 0.2;// Moment of inertia around z-axis

    qdot.resize(q.size(),0);

    qdot[0]  = dot_x;                      //vel_x
    qdot[1]  = dot_y;                      //vel_y
    qdot[2]  = dot_psi;                    //ang_yaw_vel
    qdot[3]  = (u[0]/m) * sin(psi);           //acc_x
    qdot[4]  = -(u[0]/m) * cos(psi);          //acc_y
    qdot[5]  = u[1] / I_z;                  //ang_yaw_acc

}

void makeStreet(std::vector<Rectangle> & obstacles/* obstacles */)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obstacle1;
	obstacle1.x = 0;
	obstacle1.y = 30;
	obstacle1.width = 1;
	obstacle1.height = 10;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 10;
	obstacle2.y = 20;
	obstacle2.width = 1;
	obstacle2.height = 1;
	obstacles.push_back(obstacle2);

    Rectangle obstacle3;
	obstacle3.x = 10;
	obstacle3.y = 10;
	obstacle3.width = 1;
	obstacle3.height = 1;
	obstacles.push_back(obstacle3);

	Rectangle obstacle4;
	obstacle4.x = 20;
	obstacle4.y = 20;
	obstacle4.width = 1;
	obstacle4.height = 1;
	obstacles.push_back(obstacle4);

    Rectangle obstacle5;
	obstacle5.x = 20;
	obstacle5.y = 9;
	obstacle5.width = 1;
	obstacle5.height = 1;
	obstacles.push_back(obstacle5);

    Rectangle obstacle6;
	obstacle6.x = 30;
	obstacle6.y = 30;
	obstacle6.width = 1;
	obstacle6.height = 10;
	obstacles.push_back(obstacle6);

    Rectangle obstacle7;
	obstacle7.x = 30;
	obstacle7.y = 19;
	obstacle7.width = 10;
	obstacle7.height = 1;
	obstacles.push_back(obstacle7);

    Rectangle obstacle8;
	obstacle8.x = 40;
	obstacle8.y = 29;
	obstacle8.width = 1;
	obstacle8.height = 1;
	obstacles.push_back(obstacle8);

    Rectangle obstacle9;
	obstacle9.x = 40;
	obstacle9.y = 29;
	obstacle9.width = 1;
	obstacle9.height = 1;
	obstacles.push_back(obstacle9);

    std::ofstream output("obstacles.txt");
    for (const auto& obstacle : obstacles) {
        output << obstacle.x << " " 
                   << obstacle.y << " " 
                   << obstacle.width << " " 
                   << obstacle.height << std::endl;
    }
    output.close();
}

//State Validity Checker for checking if square voxel of the drone is in collision
bool isValidStateSquare(const ompl::base::State *state, const std::vector<Rectangle> &obstacles, const ompl::control::SpaceInformationPtr si)
{
    const auto *cpace = si->getStateSpace()->as<ompl::base::CompoundStateSpace>();
    const auto *space = cpace->as<ompl::base::RealVectorStateSpace>(0);

    auto cstate = state->as<ompl::base::CompoundStateSpace::StateType>();
    auto r2 = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    auto yaw = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    auto dot_vector = cstate->as<ompl::base::RealVectorStateSpace::StateType>(2);

    double x = r2->values[0];
    double y = r2->values[1];
    double yaw_angle = yaw->value;

    double dot_x = dot_vector->values[0];
    double dot_y = dot_vector->values[1];
    double dot_yaw = dot_vector->values[2];
    
    double sideLength = 0.3;

    double hbound_x = space->getBounds().high[0];
    double lbound_x = space->getBounds().low[0];
    double hbound_y = space->getBounds().high[1];
    double lbound_y = space->getBounds().low[1];
    double bound_dot_x = 0.5;
    double bound_dot_y = 0.5;
    double bound_dot_yaw = 1.0;

    if(dot_x > bound_dot_x)
    {
        return false;

    }
    else if(dot_x < -bound_dot_x)
    {  
        return false;
    }

    if(dot_y > bound_dot_y)
    {
        return false;

    }
    else if(dot_y < -bound_dot_y)
    {  
        return false;
    }

    if(dot_yaw > bound_dot_yaw)
    {
        return false;

    }
    else if(dot_yaw < -bound_dot_yaw)
    {  
        return false;
    }

    if (x < lbound_x || x > hbound_x || y < lbound_y || y > hbound_y)
    {
        return false;
    }   
    bool is_ok  = isValidSquare(x, y, yaw_angle, sideLength, obstacles);//isValidPoint(x, y, obstacles);

    return is_ok;//isValidSquare(x, y, theta, sideLength, obstacles);
}

ompl::control::SimpleSetupPtr createDrone(std::vector<Rectangle> &obstacles /* obstacles */)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    auto r2_space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0,0);
    bounds.setHigh(0,50.0);
    bounds.setLow(1,0);
    bounds.setHigh(1,40.0);
    r2_space->setBounds(bounds);

    auto yaw_space = std::make_shared<ompl::base::SO2StateSpace>();
    auto dot_vector_space = std::make_shared<ompl::base::RealVectorStateSpace>(3);//dot_x, dot_y, dot_yaw
    ompl::base::RealVectorBounds dot_bounds(3);
    dot_bounds.setLow(0,-0.5);
    dot_bounds.setHigh(0,0.5);
    dot_bounds.setLow(1,-0.5);
    dot_bounds.setHigh(1,0.5);
    dot_bounds.setLow(2,-1.0);
    dot_bounds.setHigh(2,1.0);
    dot_vector_space->setBounds(dot_bounds);

    space->addSubspace(r2_space, 1.0);
    space->addSubspace(yaw_space, 1.0);
    space->addSubspace(dot_vector_space, 1.0);


    auto con_space = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);

    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -10.0);  // Thrust lower bound
    cbounds.setHigh(0, 10.0);  // Thrust upper bound
    cbounds.setLow(1, -1.0);   // Z_Moment lower bound
    cbounds.setHigh(1, 1.0);   // Z_Moment upper bound
    con_space->setBounds(cbounds);

    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(con_space);
    
    ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<>(ss->getSpaceInformation(), &droneODE));

    /*set the state propagation routine*/
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, postPropagate));

    ss->setStateValidityChecker([=](const ompl::base::State* state) {
        return isValidStateSquare(state, obstacles, ss->getSpaceInformation());
        });

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);   
    start->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = 10.0;  // x
    start->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = 35.0;  // y
    start->as<ompl::base::SO2StateSpace::StateType>(1)->value            = 0.0;  // yaw
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0] = 0.0;  // dot_x
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1] = 0.0;  // dot_y
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[2] = 0.0;  // dot_yaw

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);   
    goal->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = 45.0;    // x
    goal->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = 35.0;    // y
    goal->as<ompl::base::SO2StateSpace::StateType>(1)->value            = 0.0;    // yaw
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0] = 0.0;  // dot_x
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1] = 0.0;  // dot_y
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[2] = 0.0;  // dot_yaw
  
    ss->setStartAndGoalStates(start, goal, 3.0);
    return ss;
}

ompl::control::PathControl planDrone(ompl::control::SimpleSetupPtr &ss/* ss */)
{
    // TODO: Do some motion planning for the Drone
    ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    ss->getSpaceInformation()->setPropagationStepSize(0.2);
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(10.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.interpolate();
        return path; // Return the solution path
    }
    else
        std::cout << "No solution found" << std::endl;
        return ompl::control::PathControl(ss->getSpaceInformation());

}

//Calculate the Euclidean Distance between the start state and all other states
void CalculateDistances(double x, double y, const std::vector<std::pair<double, double>> &coords, std::vector<double> &dist_vec)
{
    for (const auto &coord : coords)
    {
        double x2 = coord.first;
        double y2 = coord.second;

        // Calculate Euclidean distance squared
        double distanceSquared = (x - x2) * (x - x2) + (y - y2) * (y - y2);
        dist_vec.push_back(distanceSquared);
    }
    
}

//Obtain the newStartState from the Path
std::pair<size_t, ompl::base::State *> getNewStart(ompl::control::PathControl &path)
{
    // Get all states in the path
    const std::vector<ompl::base::State *> &states = path.getStates();
    std::vector<std::pair<double, double>> coords;
    std::vector<double> dist_vec;
    double index = 0; 

    // Iterate through each state
    for (size_t i = 0; i < states.size(); i++)
    {
        // State space is CompoundStateSpace (R^2 + SO(2))
        const auto *compoundState = states[i]->as<ompl::base::CompoundStateSpace::StateType>();

        // Access R^2 component (x, y)
        const auto *r2 = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(0);
        double x = r2->values[0];
        double y = r2->values[1];
        coords.emplace_back(x,y);
    }

    if (!coords.empty())
    {
        double x1 = coords[0].first;
        double y1 = coords[0].second;
        std::cout << "x1: " << x1 << std::endl;
        std::cout << "y1: " << y1 << std::endl;
        CalculateDistances(x1, y1, coords, dist_vec);

        // Find the first index within visible region
        for (size_t i = 0; i < dist_vec.size(); ++i)
        {
            if (dist_vec[i] < VISIBLE_REGION*VISIBLE_REGION)
            {
                index = i;
            }
        }
    }
    std::cout << "Index: " << index << std::endl;
    return {index, states[index]};
}

//Calculate the distance between Start and Goal State
double calculateDistanceBetweenStartAndGoal(const ompl::control::SimpleSetupPtr &ss)
{
    // Get the problem definition
    auto pdef = ss->getProblemDefinition();

    // Retrieve the start state (assuming there's only one start state)
    const ompl::base::State *startState = pdef->getStartState(0);

    // Retrieve the goal state (assuming it's a GoalState)
    const ompl::base::Goal *goal = pdef->getGoal().get();
    const auto *goalState = goal->as<ompl::base::GoalState>();

    if (!goalState)
    {
        std::cerr << "Goal is not a single state!" << std::endl;
        return -1.0; // Error case
    }

    const ompl::base::State *goalStateRaw = goalState->getState();

    // Calculate the distance using SpaceInformation
    double distance = ss->getSpaceInformation()->distance(startState, goalStateRaw);

    return distance;
}

void appendIndexLinesToFile(const ompl::control::PathControl &path, const std::string &filename, size_t index, bool overwrite = false)
{
    // Create a stringstream to store the path as a matrix
    std::stringstream ss;
    path.asGeometric().printAsMatrix(ss); // Print the entire path as a matrix into the stream

    // Open the file in append mode
    std::ofstream output(filename, std::ios::app);
    if (!output.is_open())
    {
        std::cerr << "Error: Unable to open " << filename << " for appending!" << std::endl;
        return;
    }

    // Read the stream line by line and append only "index" number of lines
    std::string line;
    size_t count = 0;
    while (std::getline(ss, line))
    {
        if (count >= index) // Stop after writing "index" lines
            break;

        output << line << "\n"; // Append the line to the file
        std::cout << line << std::endl;
        count++;
    }

    output.close(); // Close the file
}


int main()
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);
    ompl::control::SimpleSetupPtr ss = createDrone(obstacles);

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> startState(ss->getSpaceInformation()->getStateSpace());  
    auto pdef = ss->getProblemDefinition();
    startState = pdef->getStartState(0);
    double distance = -1.0;
    double count_loop = 0;

    const std::string filename = "path.txt";

    // Check if the file exists
    if (std::remove(filename.c_str()) == 0)
    {
        // Remove the file
        std::cout << "File '" << filename << "' has been removed." << std::endl;
    }
    else
    {
        //Print a message
        std::cout << "Path will be saved in -- path.txt -- file" << std::endl;
    }

    do
    {
        // Planning
        ompl::control::PathControl path = planDrone(ss);
    
        if (!path.getStates().empty())
        {
            std::pair<size_t, ompl::base::State *> result = getNewStart(path);
            size_t index = result.first;
            appendIndexLinesToFile(path, filename, index);
            ompl::base::State *newStartRaw = result.second;
            std::cout << "NewStartState" << newStartRaw << std::endl;
            /////////////////////////////////////////////////////////
            auto *compoundState = newStartRaw->as<ompl::base::CompoundStateSpace::StateType>();

            // Access R^2 component (position: x, y)
            const auto *r2 = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(0);
            double x = r2->values[0];
            double y = r2->values[1];
            const auto *yawState = compoundState->as<ompl::base::SO2StateSpace::StateType>(1);
            double yaw = yawState->value;
            const auto *dot_vector = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(2);
            double dot_x = dot_vector->values[0];
            double dot_y = dot_vector->values[1];
            double dot_yaw = dot_vector->values[2];
            std::cout << "X: "<< x << std::endl;
            std::cout << "Y: "<< y << std::endl;
            std::cout << "Yaw: " << yaw << std::endl;
            std::cout << "DotX: "<< dot_x << std::endl;
            std::cout << "DotY: "<< dot_y << std::endl;
            std::cout << "DotYaw: " << dot_yaw << std::endl;
            /////////////////////////////////////////////////////////

            if (newStartRaw != nullptr)
            {
                ompl::base::ScopedState<ompl::base::CompoundStateSpace> newStart(ss->getSpaceInformation()->getStateSpace());
                newStart = newStartRaw;
                // Set the newStartState as startState
                startState = newStart;
                std::cout << "New start state has been set." << std::endl;
                
                //New Start State
                ss->setStartState(startState);

            }
            else
            {
                std::cout << "No valid new start state found." << std::endl;
            }
            
        }
        else
        {
            std::cout << "No valid new start state found." << std::endl;
            break;
        }
        // Calculate the distance between Start and Goal states
        distance = calculateDistanceBetweenStartAndGoal(ss);
        std::cout << "Distance between start and goal:" << distance <<std::endl;
        count_loop ++;
        std::cout << "Loop Number: " << count_loop << std::endl;
    } while (distance > MOVE_DISTANCE);

    // Benchmarking
    // benchmarkCar(ss);

    return 0;
}