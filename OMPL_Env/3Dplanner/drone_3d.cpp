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
#include <tuple>
#include <random>

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

std::random_device rd;
std::default_random_engine roll_val(rd());
std::normal_distribution<double> roll_space(0.0, 0.3);


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
    double roll = roll_space(roll_val);

    // State Space: {x,y,psi,dot_x,dot_y,dot_psi}
    const double *u  = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double F   = u[0];               //Total thrust
    const double Mz = u[1];                //Torque around z-axis

    const double psi       = q[3];         //yaw
    const double dot_x     = q[4];         //vel_x
    const double dot_y     = q[5];         //vel_y
    const double dot_z     = q[6];         //vel_z
    const double dot_psi   = q[7];         //ang_yaw_vel

    // Constants
    const double m = 1.0;  // Mass of the quadrotor
    const double g = 9.81; // Acceleration due to gravity
    const double I_z = 0.2;// Moment of inertia around z-axis

    qdot.resize(q.size(),0);

    qdot[0]  = dot_x;                      //vel_x
    qdot[1]  = dot_y;                      //vel_y
    qdot[2]  = dot_z;                      //vel_z
    qdot[3]  = dot_psi;                    //ang_yaw_vel
    qdot[4]  = (u[0]/m) * sin(psi);        //acc_x
    qdot[5]  = (u[0]/m) * sin(psi) + roll; //acc_y
    qdot[6]  = (u[0]/m) - g;              //acc_z
    qdot[7]  = u[1] / I_z;                 //ang_yaw_acc

}

void makeStreet(std::vector<Rectangle> & obstacles/* obstacles */)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle obstacle1;
	obstacle1.x = 0;
	obstacle1.y = 30;
    obstacle1.z = 0;
	obstacle1.length = 1;
	obstacle1.breadth = 10;
    obstacle1.height = 10;
	obstacles.push_back(obstacle1);

	Rectangle obstacle2;
	obstacle2.x = 10;
	obstacle2.y = 20;
    obstacle2.z = 0;
	obstacle2.length = 1;
	obstacle2.breadth = 1;
    obstacle2.height = 15;
	obstacles.push_back(obstacle2);

    Rectangle obstacle3;
	obstacle3.x = 10;
	obstacle3.y = 10;
    obstacle3.z = 0;
	obstacle3.length = 1;
	obstacle3.breadth = 1;
    obstacle3.height = 20;
	obstacles.push_back(obstacle3);

	Rectangle obstacle4;
	obstacle4.x = 20;
	obstacle4.y = 20;
    obstacle4.z = 0;
	obstacle4.length = 1;
	obstacle4.breadth = 1;
    obstacle4.height = 20;
	obstacles.push_back(obstacle4);

    Rectangle obstacle5;
	obstacle5.x = 20;
	obstacle5.y = 9;
    obstacle5.z = 0;
	obstacle5.length = 1;
	obstacle5.breadth = 1;
    obstacle5.height = 15;
	obstacles.push_back(obstacle5);

    Rectangle obstacle6;
	obstacle6.x = 30;
	obstacle6.y = 30;
    obstacle6.z = 0;
	obstacle6.length = 1;
	obstacle6.breadth = 10;
    obstacle6.height = 15;
	obstacles.push_back(obstacle6);

    Rectangle obstacle7;
	obstacle7.x = 30;
	obstacle7.y = 19;
    obstacle7.z = 0;
	obstacle7.length = 10;
	obstacle7.breadth = 1;
    obstacle7.height = 20;
	obstacles.push_back(obstacle7);

    Rectangle obstacle8;
	obstacle8.x = 40;
	obstacle8.y = 29;
    obstacle8.z = 0;
	obstacle8.length = 1;
	obstacle8.breadth = 1;
    obstacle8.height = 15;
	obstacles.push_back(obstacle8);

    Rectangle obstacle9;
	obstacle9.x = 40;
	obstacle9.y = 29;
    obstacle9.z = 0;
	obstacle9.length = 1;
	obstacle9.breadth = 1;
    obstacle9.height = 20;
	obstacles.push_back(obstacle9);

    std::ofstream output("obstacles.txt");
    for (const auto& obstacle : obstacles) {
        output << obstacle.x << " " 
                   << obstacle.y << " " 
                   << obstacle.z << " "
                   << obstacle.length << " " 
                   << obstacle.breadth << " "
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
    auto r3 = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    auto yaw = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    auto dot_vector = cstate->as<ompl::base::RealVectorStateSpace::StateType>(2);

    double x = r3->values[0];
    double y = r3->values[1];
    double z = r3->values[2];
    double yaw_angle = yaw->value;

    double dot_x = dot_vector->values[0];
    double dot_y = dot_vector->values[1];
    double dot_z = dot_vector->values[2];
    double dot_yaw = dot_vector->values[3];
    
    double sideLength = 0.3;

    double hbound_x = space->getBounds().high[0];
    double lbound_x = space->getBounds().low[0];
    double hbound_y = space->getBounds().high[1];
    double lbound_y = space->getBounds().low[1];
    double hbound_z = space->getBounds().high[2];
    double lbound_z = space->getBounds().low[2];
    double bound_dot_x = 0.5;
    double bound_dot_y = 0.5;
    double bound_dot_z = 1.0;
    double bound_dot_yaw = 1.0;

    /////////////////////////////////////////////////////////////////////////////////////
    // std::cout << "----------------------Collision Checker-----------------" << std::endl;
    // std::cout << "x: " << x << std::endl;
    // std::cout << "y: " << y << std::endl;
    // std::cout << "z: " << z << std::endl;
    // std::cout << "yaw: " << yaw_angle << std::endl;
    // std::cout << "x_dot: " << dot_x << std::endl;
    // std::cout << "y_dot: " << dot_y << std::endl;
    // std::cout << "z_dot: " << dot_z << std::endl;
    // std::cout << "yaw_dot: " << dot_yaw << std::endl;
    // std::cout << "----------------------END-----------------" << std::endl;
    /////////////////////////////////////////////////////////////////////////////////////


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

    if(dot_z > bound_dot_z)
    {
        return false;

    }
    else if(dot_z < -bound_dot_z)
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

    if (x < lbound_x || x > hbound_x || y < lbound_y || y > hbound_y || z < lbound_z || z > hbound_z)
    {
        return false;
    }   
    bool is_ok  = isValidSquare(x, y, z, yaw_angle, sideLength, obstacles);//isValidPoint(x, y, obstacles);

    return is_ok;//isValidSquare(x, y, theta, sideLength, obstacles);
}

ompl::control::SimpleSetupPtr createDrone(std::vector<Rectangle> &obstacles /* obstacles */)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    auto r3_space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0,0.0);
    bounds.setHigh(0,50.0);
    bounds.setLow(1,0.0);
    bounds.setHigh(1,40.0);
    bounds.setLow(2,0.0);
    bounds.setHigh(2,20.0);
    r3_space->setBounds(bounds);

    auto yaw_space = std::make_shared<ompl::base::SO2StateSpace>();
    auto dot_vector_space = std::make_shared<ompl::base::RealVectorStateSpace>(4);//dot_x, dot_y, dot_z, dot_yaw
    ompl::base::RealVectorBounds dot_bounds(4);
    dot_bounds.setLow(0,-0.5);
    dot_bounds.setHigh(0,0.5);
    dot_bounds.setLow(1,-0.5);
    dot_bounds.setHigh(1,0.5);
    dot_bounds.setLow(2,-1.0);
    dot_bounds.setHigh(2,1.0);
    dot_bounds.setLow(3,-1.0);
    dot_bounds.setHigh(3,1.0);
    dot_vector_space->setBounds(dot_bounds);

    space->addSubspace(r3_space, 1.0);
    space->addSubspace(yaw_space, 1.0);
    space->addSubspace(dot_vector_space, 1.0);

    auto con_space = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);

    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(0, 0.0);  // Thrust lower bound
    cbounds.setHigh(0, 15.0);  // Thrust upper bound
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
    start->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = 5.0;  // x
    start->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = 5.0;  // y
    start->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = 5.0;  // z
    start->as<ompl::base::SO2StateSpace::StateType>(1)->value            = 0.0;  // yaw
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0] = 0.0;  // dot_x
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1] = 0.0;  // dot_y
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[2] = 0.0;  // dot_z
    start->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[3] = 0.0;  // dot_yaw

    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);   
    goal->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0] = 45.0;    // x
    goal->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[1] = 35.0;    // y
    goal->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[2] = 15.0;    // z
    goal->as<ompl::base::SO2StateSpace::StateType>(1)->value            = 0.0;    // yaw
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[0] = 0.0;  // dot_x
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[1] = 0.0;  // dot_y
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[2] = 0.0;  // dot_z
    goal->as<ompl::base::RealVectorStateSpace::StateType>(2)->values[3] = 0.0;  // dot_yaw
  
    ss->setStartAndGoalStates(start, goal, 3.0);
    return ss;
}

ompl::control::PathControl planDrone(ompl::control::SimpleSetupPtr &ss/* ss */)
{
    // TODO: Do some motion planning for the Drone
    ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    ss->getSpaceInformation()->setPropagationStepSize(0.2);
    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(100.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ompl::control::PathControl &path = ss->getSolutionPath();
        path.interpolate();
        // path.asGeometric().printAsMatrix(std::cout);
        // std::ofstream output("path.txt");
        // path.asGeometric().printAsMatrix(output);
        // output.close();
        return path; // Return the solution path
    }
    else
        std::cout << "No solution found" << std::endl;
        return ompl::control::PathControl(ss->getSpaceInformation());

}

//Calculate the Euclidean Distance between the start state and all other states
void CalculateDistances(double x, double y, double z, const std::vector<std::tuple<double, double, double>> &coords, std::vector<double> &dist_vec)
{
    for (const auto &coord : coords)
    {
        double x2 = std::get<0>(coord);
        double y2 = std::get<1>(coord);
        double z2 = std::get<2>(coord);

        // Calculate Euclidean distance squared
        double distanceSquared = (x - x2) * (x - x2) + (y - y2) * (y - y2) + (z - z2)*(z - z2);
        dist_vec.push_back(distanceSquared);
    }
    
}

//Obtain the newStartState from the Path
std::pair<size_t, ompl::base::State *> getNewStart(ompl::control::PathControl &path)
{
    // Get all states in the path
    const std::vector<ompl::base::State *> &states = path.getStates();
    std::vector<std::tuple<double, double, double>> coords;
    std::vector<double> dist_vec;
    size_t index = 0; 

    // Iterate through each state
    for (size_t i = 0; i < states.size(); i++)
    {
        // State space is CompoundStateSpace (R^3 + SO(2) + R^4)
        const auto *compoundState = states[i]->as<ompl::base::CompoundStateSpace::StateType>();

        // Access R^3 component (x, y, z)
        const auto *r3 = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(0);
        double x = r3->values[0];
        double y = r3->values[1];
        double z = r3->values[2];
        coords.emplace_back(x,y,z);
    }

    if (!coords.empty())
    {
        double x1 = std::get<0>(coords[0]);
        double y1 = std::get<1>(coords[0]);
        double z1 = std::get<2>(coords[0]);
        std::cout << "x1: " << x1 << std::endl;
        std::cout << "y1: " << y1 << std::endl;
        std::cout << "z1: " << z1 << std::endl;
        CalculateDistances(x1, y1, z1, coords, dist_vec);

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
        std::cout << "File '" << filename << "' will be overwritten." << std::endl;
    }
    else
    {
        //Print a message
        std::cout << "Path will be saved in -- " << filename << " -- file" << std::endl;
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

            // Access R^2 component (position: x, y, z)
            const auto *r3 = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(0);
            double x = r3->values[0];
            double y = r3->values[1];
            double z = r3->values[2];
            const auto *yawState = compoundState->as<ompl::base::SO2StateSpace::StateType>(1);
            double yaw = yawState->value;
            const auto *dot_vector = compoundState->as<ompl::base::RealVectorStateSpace::StateType>(2);
            double dot_x = dot_vector->values[0];
            double dot_y = dot_vector->values[1];
            double dot_z = dot_vector->values[2];
            double dot_yaw = dot_vector->values[3];
            std::cout << "X: "<< x << std::endl;
            std::cout << "Y: "<< y << std::endl;
            std::cout << "Z: "<< z << std::endl;
            std::cout << "Yaw: " << yaw << std::endl;
            std::cout << "DotX: "<< dot_x << std::endl;
            std::cout << "DotY: "<< dot_y << std::endl;
            std::cout << "DotZ: "<< dot_z << std::endl;
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