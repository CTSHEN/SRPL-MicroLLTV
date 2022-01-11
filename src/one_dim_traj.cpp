/***********************************************************
 * File Name: one_dim_traj.cpp
 * 1-D Optimal Trajectory v0.1
 * This program is associate with one_dim_ocp.
 * This program generates the optimal reference trajectory
 * for one_dim_ocp.
 * This progrma updates a new trajectory based on state estimation 
 * every second.
 * *********************************************************/

#include "psopt.h"
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#define M2 8 // kg
#define M1S 8.5 //kg
#define MF_INIT 0.5 //kg
#define G 9.81 // m/s2
#define THRUST 15 // N
#define FLOW_RATE 0.018 // kg/s

#define DIS_X  1.5//0.3
#define DIS_V  0

#define USE_SIM_TIME true

#define TRAJ_PLAN_HZ 2.0 // planning frequency
#define DELTA_T 0.2    //s, control step size
#define PLAN_HORIZON 2.5 //s
#define STATE_HEIGHT 1 // define state index 1 as height
#define STATE_VELOCITY 2
#define STATE_FUELMASS 3

using namespace PSOPT;

///////// Define Global Variables //////////
double position = 0.0;
double position_stamped;
double velocity = 0.0;
double velocity_stamped;
double FmassEst = MF_INIT;

float lastest_control;

///////// PSOPT configuration //////////

////////// Define the endpoint cost function//////////
adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters, adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
    return tf;
}

////////// Define the Lagrange cost function //////////
adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
    return 0;
}

///////// Define system dynamics //////////
void dae(adouble* derivatives, adouble* path, adouble* states, adouble* controls, 
         adouble* patameters, adouble& time, adouble* xad, int iphase, Workspace* workspace)
         {
            adouble xdot, vdot, mdot;

            adouble x = states[0];
            adouble v = states[1];
            adouble m = states[2];

            adouble u = controls[0];

            xdot = v;
            vdot = (M2-M1S-m)*G/(M2+M1S+m) + THRUST*u/(M2+M1S+m);
            mdot = -FLOW_RATE*smooth_fabs(u, 0.01);

            derivatives[0] = xdot;
            derivatives[1] = vdot;
            derivatives[2] = mdot;                   
         }

////////// Define the event function //////////
void events(adouble* e, adouble* initial_states, adouble* final_states,
            adouble* parameters, adouble& t0, adouble& tf, adouble* xad,
            int iphase, Workspace* workspace)
{
    adouble x0 = initial_states[0];
    adouble v0 = initial_states[1];
    adouble m0 = initial_states[2];
    adouble xf = final_states[0];
    adouble vf = final_states[1];
    

    e[0] = x0;
    e[1] = v0;
    e[2] = m0;
    e[3] = xf;
    e[4] = vf;
    
}

void linkages( adouble* linkages, adouble* xad, Workspace* workspace)
{

}

/////////////////////////////////////
////////// ROS Config Part //////////
/////////////////////////////////////

void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    position = msg->pose.position.x; 
    position_stamped = msg -> header.stamp.toSec();
} 

void velocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    velocity = msg->twist.linear.x;
    velocity_stamped = msg -> header.stamp.toSec();
}

void massCb(const std_msgs::Float32::ConstPtr& msg)
{
    FmassEst = msg->data;
}

void controlCb(const std_msgs::Float32::ConstPtr& msg)
{
    lastest_control = msg->data; 

}

////////// Trajectory Optimization class definition //////////
class TrajOpt
{

    public:
        TrajOpt(ros::NodeHandle *nh,
            Prob &problemIn, Sol &solutionIn, Alg &algorithmIn)
        {
            last_control = 0.0;
            rk4_control.resize(1,2);
            rk4_time.resize(1,2);
            rk4_states.resize(3,2);
            init_states.resize(3,1);

            //interpT.resize(1,13);

            problem = problemIn;
            solution = solutionIn;
            algorithm = algorithmIn;

            //Publish trajectory
            path_pub = nh->advertise<nav_msgs::Path>
                ("optTraj", 1);

            // Timer initialization (Default Duration: 1s)
            // oneshot = true, autostart = false
            timeToPub = nh->createTimer(ros::Duration(1),
                &TrajOpt::PubTrajCb , this, true, false); 

        }

        double curr_Time;
        adouble curr_Time_a;
        adouble& curr_Time_ref = curr_Time_a;
        double stamp_now, stamp_next;
        Prob problem;
        Sol solution;
        Alg algorithm;
        //adouble interpTime[13] = {0, 0.2, 0.4, 0.6, 0.8, 1.0,
        //                        1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4};
        //adouble interpHeight[13] = {0};
        //adouble interpVelcity[13];
        adouble interpScale = 1;
        MatrixXd interpH, interpV, interpT;


        // vectors for rk4 propagate
        MatrixXd rk4_control, rk4_time, rk4_states, rk4_param, init_states;
        
        //* T-0.2: start to generate optimal path which will be applied at T.
        //  (Step 1) Get current time(T-0.2) and define the publish time (T).
        //  (Step 2) Use RK4 to propagate states(T-0.2) to states(T) w/ current control.
        //  (Step 3) Use RK4 to generate initial guess from T -> T+2.5
        //  (Step 4) Generate the optimal trajectory with initial guess.
        //  (Step 5) Interpolate height and velocity at each control time instance
        //      (T, T+0.2, T+0.4, ..., T+2.4)
        //      and stack them into path message.
        //  (Step 6) get current time and create a timer with duration (T-T_current)
        //  (Step 7) Wait for the publish time and publish the new path.
        void GenNextPath(const ros::TimerEvent& event)
        {
            // Step 1 //
            curr_Time = ros::Time::now().toSec(); // get current time
            curr_Time_a = (adouble)curr_Time;
            curr_Time_ref = curr_Time_a;

            stamp_now = position_stamped;  // define T-0.2
            stamp_next = stamp_now + DELTA_T; // define T

            // Step 2 //
            rk4_control(0,0) = lastest_control; rk4_control(0,1) = lastest_control;
            rk4_time(0,0) = stamp_now; rk4_time(0,1) = stamp_next;
            init_states(0,0) = position;
            init_states(1,0) = velocity;
            init_states(2,0) = FmassEst;
            /// FOR DEBUG ///
            printf("stamp_now = %f \n", stamp_now);
            printf("lastest_control = %f \n", lastest_control);
            printf("height = %f \n", position);
            printf("velocity = %f \n", velocity);
            //////////            

            rk4_propagate(dae, rk4_control, rk4_time,
                init_states, rk4_param, problem, 1, rk4_states, NULL);

            /// FOR DEBUG ///
            printf("STATE PROPAGATED! \n");
            /////////////////

            // Step 3 //
            int nnodes = problem.phases(1).nodes(0);
            MatrixXd x0(3,nnodes);
            MatrixXd init_state_guess(3,1);
            init_state_guess(0,0) = rk4_states(0,1);
            init_state_guess(1,0) = rk4_states(1,1);
            init_state_guess(2,0) = rk4_states(2,1);
            MatrixXd& init_state_guess_ref = init_state_guess;

            problem.phases(1).guess.controls = lastest_control*ones(1,nnodes);
            problem.phases(1).guess.time = linspace(stamp_next, stamp_next+PLAN_HORIZON,nnodes);

            rk4_propagate(dae, problem.phases(1).guess.controls,
                problem.phases(1).guess.time, init_state_guess_ref, rk4_param, 
                problem, 1, x0, NULL);

            problem.phases(1).guess.states = x0;
            /// FOR DEBUG ///
            printf("INITIAL GUESS GENERATED! \n");
            /////////////////

            // Step 4 //
            ////////// problem bounds iinformation //////////
            problem.phases(1).bounds.lower.states << 0, -2, 0;
            problem.phases(1).bounds.upper.states << 3, 2, MF_INIT;

            problem.phases(1).bounds.lower.controls << -1.0;
            problem.phases(1).bounds.upper.controls << 1.0;

            problem.phases(1).bounds.lower.events << position, velocity, FmassEst, 0.5, 0;
            problem.phases(1).bounds.upper.events << position, velocity, FmassEst, 0.5, 0;

            problem.phases(1).bounds.lower.StartTime = stamp_next; 
            problem.phases(1).bounds.upper.StartTime = stamp_next;

            problem.phases(1).bounds.lower.EndTime = stamp_next+0;
            problem.phases(1).bounds.upper.EndTime = stamp_next+2.5;

            ////////// Call PSOPT to solve the problem /////////
            psopt(solution, problem, algorithm);
            if(solution.error_flag) { printf("ERROR FLAG!"); exit(0);}

            // Step 5 //
            
            ////////// Get states and time //////////
            MatrixXd xStar = solution.get_states_in_phase(1);
            MatrixXd HStar = xStar.row(0);
            MatrixXd VStar = xStar.row(1);
            MatrixXd t_sol = solution.get_time_in_phase(1);
            /****************DEBUG INFORMATION****************************/
            //plot(t_sol,xStar,problem.name + ":states", "times(s)", "states", "x v m");
            //plot(t_sol,uStar,problem.name + ": control", "time (s)", "control", "u");
            /*************************************************************/
            
            printf("I AM HERE~ \n"); /// FOR DEBUG ///
            // Clear last path
            optTraj.poses.clear();
                        
            ////////// Interpolate states with interpTime //////////
            // Generate interpolate time
            //printf(" last t_sol is %f \n", t_sol.row(0).tail(1).value()); // FOR DEBUG
            interpT.resize(1,(int)((t_sol.row(0).tail(1).value()-stamp_next)/0.2));
            //printf("size of interpT = %d", interpT.cols()); //FOR DEBUG
            for (int i = 0; i< interpT.cols(); i++)
            {
                // There are 13 collocation points in default, if there are
                // points greater than the time of the optimal solution,
                // drop them.
                //if(stamp_next+0.2*i > t_sol.row(0).tail(1).value()) break;
                interpT(0,i) = stamp_next + 0.2*i;
                //printf("GET TIME !\n ");  // FOR DEBUG
            }

            // get interpolated height
            lagrange_interpolation(interpH, interpT, t_sol, HStar);
            // get interpolated velocity
            lagrange_interpolation(interpV, interpT, t_sol, VStar);
            ////////// DEBUG INFORMATION //////////
            /*plot(t_sol, xStar.row(0), interpT, interpH, "Height sol and interpolation",
                 "time (s)", "Height(m)", "sol interpH");
            
            plot(t_sol, xStar.row(1), interpT, interpV, "Velocity sol and interpolation",
                 "time (s)", "Velocity(m/s)", "sol interpV");*/
            ///////////////////////////////////////
            
            for (int i =0; i< interpT.size(); i++)
            {
                // assign height, velocity and time into posestamped and pushback to path
                // poses.pose.position.x ---> velocity
                // poses.pose.position.z ---> height
                pose2PushBack.header.seq = i+1;
                pose2PushBack.header.stamp.sec = interpT(0,i);
                pose2PushBack.header.stamp.nsec = fmod(interpT(0,i),1)*1e9;
                pose2PushBack.pose.position.x = interpH(0,i);
                pose2PushBack.pose.position.z = interpV(0,i);
                printf("trajT %d.%d\t trajH %f\t trajV %f \n",
                     pose2PushBack.header.stamp.sec,
                     pose2PushBack.header.stamp.nsec,
                     pose2PushBack.pose.position.x, 
                     pose2PushBack.pose.position.z);
                
                //pushback to path
                optTraj.poses.push_back(pose2PushBack);
            }
            


            // Step 6 //
            double time_now = ros::Time::now().toSec();
            timeToPub.setPeriod(ros::Duration(stamp_next-time_now), true);
            // Step 7 //
            timeToPub.start(); //start the timer        


        }

        void PubTrajCb(const ros::TimerEvent& event)
        {
            optTraj.header.stamp = ros::Time::now();
            path_pub.publish(optTraj);
            /*************** DEBUG INFORMATION*****************/
            //plot(interpT, interpH, " Planned Trajectory", "time(s)", "Height(m)", "x");
            /**************************************************/

            timeToPub.stop();
        }


    private:
        ros::Publisher path_pub;
        ros::Timer timeToPub;
                
        float last_control;
        nav_msgs::Path optTraj;
        geometry_msgs::PoseStamped pose2PushBack;
        
        

};

////////////////////////////////
///////// Main Routine /////////
////////////////////////////////

int main(int argc, char **argv)
{
    ////////// PSOPT Setup //////////
    // declare key structures
    Alg algorithm;
    Sol solution;
    Prob problem;

    // Rigister problem name
    problem.name = "1-D OptTraj";
    //problem.outfilename = "1d.txt";

    //define problem level constants & do level 1 setup
    problem.nphases = 1;
    problem.nlinkages = 0;
    psopt_level1_setup(problem);

    // define phase related information & do level 2 setup
    problem.phases(1).nstates = 3;
    problem.phases(1).ncontrols = 1;
    problem.phases(1).nevents = 5;
    problem.phases(1).npath = 0;
    problem.phases(1).nodes <<50;
    psopt_level2_setup(problem, algorithm);

    

    ////////// Rigister problem functions //////////

    problem.integrand_cost = &integrand_cost;
    problem.endpoint_cost = &endpoint_cost;
    problem.dae = &dae;
    problem.events = &events;
    problem.linkages = &linkages;

    ////////// Algorithm options //////////
    algorithm.nlp_method = "IPOPT";
    algorithm.scaling = "automatic";
    algorithm.derivatives = "automatic";
    algorithm.nlp_iter_max = 1000;
    algorithm.nlp_tolerance = 1e-6;

    algorithm.collocation_method = "Legendre";

    //Intialize the last control signal
    //float last_control = 0.0;

    ////////// ROS Setup //////////
    ros::init(argc, argv, "one_dim_traj_node");
    ros::NodeHandle nh;

    // Use Simulation Time Setting
    nh.setParam("/use_sim_time", USE_SIM_TIME);

    // Create AsyncSpinner
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // subcribe height, velocity and mass estimate
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("height", 1, poseCb);

    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("velocity", 1, velocityCb);

    ros::Subscriber mass_sub = nh.subscribe<std_msgs::Float32>
        ("mass_est",1 , massCb);

    ros::Subscriber control_sub = nh.subscribe<std_msgs::Float32>
        ("control", 1, controlCb);

    
    // create an instance of TrajOpt
    TrajOpt trajectory_optimization_loop(&nh, problem, solution, algorithm);

    ////////// Setup control loop timer ////
    ros::Timer controlPubTimer = nh.createTimer
        (ros::Duration(1.0/TRAJ_PLAN_HZ),
        &TrajOpt::GenNextPath, &trajectory_optimization_loop);    
    

    ros::waitForShutdown();   
    
}