///////////////////////////////////////
// RBE 550
// Final Project
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
#include<math.h>
#include<thread>
#include<Eigen/Dense>

#include<ros/ros.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/CommandTOL.h>
#include<sensor_msgs/NavSatFix.h>
#include<nav_msgs/Path.h>
#include<std_msgs/Float64.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include<dynamicEDT3D/dynamicEDTOctomap.h>


/*Include the Base ompl files*/
#include<ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include<ompl/base/samplers/ObstacleBasedValidStateSampler.h>
/*Include the control ompl files*/
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/PathControl.h>
/*Include Benchmarking files*/
#include <ompl/tools/benchmark/Benchmark.h>
#include<ompl/config.h>
/*Collision Checker*/

std::random_device rd;
std::default_random_engine roll_val(rd());
std::normal_distribution<double> roll_space(0.0, 0.3);


namespace ob = ompl::base;
namespace og = ompl::geometric;



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


                                            /********************
                                            * Global Variables *
                                            *********************/
float maxDistance = 10.0;
octomap::OcTree *tree = new octomap::OcTree(0.05);
octomap::AbstractOcTree *new_tree = NULL;
octomap::OcTree *fin_tree = new octomap::OcTree(0.05);

octomap::point3d min_(0.0,0.0,0.0);
octomap::point3d max_(40,40,40);

DynamicEDTOctomap* ptr;

float droneRadius = 0.5;
bool isUpdated = false;

geometry_msgs::PoseStamped currentPose;
geometry_msgs::PoseStamped goalPose;
sensor_msgs::NavSatFix currentPose_GPS;
nav_msgs::Path path_nav;

float globalHeading;
float threshHeading = 90.0;
float currHeading;
std_msgs::Float64 global_hdg;

/*************************************************************************************************************************/


                                                /*************
                                                * Functions *
                                                *************/ 

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

bool isStateValid(const ob::State* state)
{
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();

    std::this_thread::sleep_for(ompl::time::seconds(0.0005));
    
    octomap::point3d p(pos[0], pos[1], pos[2]);
    octomap::point3d closestObst;
    float distance;


    ptr->getDistanceAndClosestObstacle(p,distance, closestObst);

    if(distance>droneRadius && min_.x()<pos[0]<max_.x() && min_.y()<pos[1]<max_.y() && 1.2<pos[2]<max_.z())//max_.z())
    {
        return true;
    }

    else
    {
        return false;
    }
    
}

                                                
std::vector<geometry_msgs::PoseStamped> plan(geometry_msgs::PoseStamped* curr, geometry_msgs::PoseStamped* target)
{
    /** Set up the configuration space **/
    auto space = std::make_shared<ompl::base::CompoundStateSpace>();

    
    auto r3space(std::make_shared<ob::RealVectorStateSpace>(3));

    ob::RealVectorBounds bounds(3);

    bounds.setLow(0,min_.x()); // x
	bounds.setHigh(0,max_.x());
	bounds.setLow(1, min_.y()); // y
	bounds.setHigh(1,max_.y());
	bounds.setLow(2,1.2);   // z
	bounds.setHigh(2,max_.z());

    r3space->setBounds(bounds);

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

    space->addSubspace(r3space, 1.0);
    space->addSubspace(yaw_space, 1.0);
    space->addSubspace(dot_vector_space, 1.0);

    auto con_space = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);

    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(0, 0.0);  // Thrust lower bound
    cbounds.setHigh(0, 15.0);  // Thrust upper bound
    cbounds.setLow(1, -1.0);   // Z_Moment lower bound
    cbounds.setHigh(1, 1.0);   // Z_Moment upper bound
    con_space->setBounds(cbounds);    

    /* Set up the SimpleSetup object to manage the backend classes */
    ompl::control::SimpleSetup ss(con_space);
    ompl::control::ODESolverPtr odeSolver(new ompl::control::ODEBasicSolver<>(ss.getSpaceInformation(), &droneODE));
    /*set the state propagation routine*/
    ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, postPropagate));

    ob::ScopedState<> start(space);
    start[0] = curr->pose.position.x;//currentPose.pose.position.x;
    start[1] = curr->pose.position.y;//currentPose.pose.position.y;
    start[2] = 1.5;
    start[3] = 0.0;
    start[4] = 0.0;
    start[5] = 0.0;
    start[6] = 0.0;
    start[7] = 0.0;

    
    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = target->pose.position.x;//goalPose.pose.position.x;
    goal[1] = target->pose.position.y;//goalPose.pose.position.y;
    goal[2] = 1.5;//goalPose.pose.position.z;
    goal[3] = 0.0;
    goal[4] = 0.0;
    goal[5] = 0.0;
    goal[6] = 0.0;
    goal[7] = 0.0;

    ss.setStateValidityChecker(isStateValid);

    ss.setStartAndGoalStates(start, goal);

    auto planner(std::make_shared<ompl::control::RRT>(ss.getSpaceInformation()));
    
    ss.setPlanner(planner);
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();

    ob::PlannerStatus solved = ss.solve(20.0);

    std::vector<geometry_msgs::PoseStamped> waypts;
    geometry_msgs::PoseStamped pose;

    if (solved)
    {
        ss.getSolutionPath().print(std::cout);
        
        ompl::control::PathControl path = ss.getSolutionPath();

        // now read the points in this path and append them in the waypoints list
        for(std::size_t idx = 0; idx<path.getStateCount(); idx++)
        {         
            const ob::RealVectorStateSpace::StateType& pos = *path.getState(idx)->as<ob::RealVectorStateSpace::StateType>();

            pose.pose.position.x = pos[0]*cos(currHeading*3.14/180.0) - pos[1]*sin(currHeading*3.14/180.0);
            pose.pose.position.y = pos[1]*cos(currHeading*3.14/180.0) + pos[0]*sin(currHeading*3.14/180.0);
            pose.pose.position.z = pos[2];            

            waypts.push_back(pose);
            path_nav.header.frame_id = "map";
            path_nav.header.stamp = ros::Time::now();
            path_nav.poses.push_back(pose);
        }

        return waypts;

    }

    else
        {
            std::cout << "No solution found" << std::endl;
            return waypts;
        }
}
               
void local_pose_cb(const geometry_msgs::PoseStamped pose)
{
    currentPose = pose; 
}


void global_pose_cb(const sensor_msgs::NavSatFix pose)
{
    currentPose_GPS = pose;
}

void goal_pose_cb(const geometry_msgs::PoseStamped pose_)
{
    goalPose.pose.position.x = pose_.pose.position.x; 
    goalPose.pose.position.y = pose_.pose.position.y; 
    goalPose.pose.position.z = 2; 
    if(goalPose.pose.position.x>0.05)
    {    
        isUpdated = true;   
        std::cout<<"New goal received"<<goalPose.pose.position.x<<std::endl;
    }
    else
        {
            isUpdated=false;
        }
}   

void global_hdg_cb(const std_msgs::Float64 heading)
{
    globalHeading = heading.data;
    currHeading = threshHeading - globalHeading;
}



                                                /********
                                                 * MAIN *
                                                 ********/ 


int main(int argc, char** argv)
{
    /*------------------------------------- ROS INITIALIZATION-------------------------------------------*/

    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh; 

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pose_cb);
    ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global",10, global_pose_cb);
    ros::Publisher  waypoint_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::Subscriber goal_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,goal_pose_cb);
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("/lucifer/freePath",10);
    ros::Subscriber global_hdg_subscriber = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg", 10, global_hdg_cb);

    /* Services for arming, landing and changing mode */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/landing");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20);

    /*--------------------------------------------------------------------------------------------------*/
    if(argc<=1)
    {
        std::cout<<"usage: "<<argv[0]<<" <octoMap.bt>"<<std::endl;
        exit(0);
    }

    tree->readBinary(argv[1]); // read the octomap in binary form

    std::cout<<"read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;

    double xmin, ymin, zmin;
    double xmax, ymax, zmax;
    tree->getMetricMin(xmin,ymin,zmin);
    octomap::point3d min(xmin,ymin,zmin);
    std::cout<<"Metric min: "<<xmin<<","<<ymin<<","<<zmin<<std::endl;
    tree->getMetricMax(xmax,ymax,zmax);
    octomap::point3d max(xmax,ymax,zmax);
    std::cout<<"Metric max: "<<xmax<<","<<ymax<<","<<zmax<<std::endl;

    bool unknownAsOccupied = false;

    DynamicEDTOctomap _distmap(maxDistance, tree, min, max, unknownAsOccupied);
    ptr = &_distmap;

    std::cout<<"Pointer allocated "<<std::endl;

    ptr->update();
    std::cout<<"EDT of the octomap calculated ..."<<std::endl;
    //_distmap.update();
    min_ = min;
    max_ = max;
    
    /**
     * For every new goal
     * Run the planner
     * get waypoints
     * traverse them
    **/
   std::vector<geometry_msgs::PoseStamped> waypoints;

    while(ros::ok())
    {
        if(isUpdated)
            {

            /** Get the waypoints
              * from the planner */
            waypoints = plan(&currentPose, &goalPose);
            if(waypoints.size()<2)
            {
                continue;
            }
            std::cout<<"Got the waypoints"<<std::endl;


            /** Publish the waypoints
              * for position control **/
            double THRESH = 0.8;

            for(int i=0; i<waypoints.size(); i++)
            {
                double distance = sqrt(pow((currentPose.pose.position.x - waypoints.at(i).pose.position.x),2) + pow((currentPose.pose.position.y - waypoints.at(i).pose.position.y),2) + pow((currentPose.pose.position.z - waypoints.at(i).pose.position.z),2));
                std::cout<<i<<std::endl;
        
                while(distance>THRESH)
                {   
                    waypoint_publisher.publish(waypoints.at(i));
                    path_publisher.publish(path_nav);
                    distance = sqrt(pow((currentPose.pose.position.x - waypoints.at(i).pose.position.x),2) + pow((currentPose.pose.position.y - waypoints.at(i).pose.position.y),2) + pow((currentPose.pose.position.z - waypoints.at(i).pose.position.z),2));
                    ros::spinOnce();
                    rate.sleep();

                    if(distance<THRESH)
                    {
                        break;
                    }
                }

                ROS_INFO("Waypoint Reached[%1d]",i);

            }
        
        ROS_INFO("Drone reached the location and loitering ...");
        isUpdated = false;
            }
        
        ros::spinOnce();
    }
    return 0;
}