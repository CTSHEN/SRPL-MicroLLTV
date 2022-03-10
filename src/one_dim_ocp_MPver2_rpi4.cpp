/**
 * @brief 1-D Hieght optimal control 
 * 
 * @version MPver2_rpi4
 * 
 * @note  MPver 1:
 *           Track a step input reference. The cost function contains position error and final
 *           time for end point cost; The position error and input penalty for integration cost.
 *        MPver2: 
 *           Consider thruster dynamics as states for planning a trajectory.
 *           The total 5 states: position, velocity, fuel mass, downward and upward thrust force.
 *        MPver2_rpi4:
 *           This version modifies the MPver2 for implementation on raspberry pi 4. The
 *           modification includes
 *              1. Output two indivisual control topics for dawnward and upward thrusters.
 *             //TODO 2. Direct output the on-off control signal to GPIO.  
 * @author Shen Chang-te
 */

#include "psopt.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>

#define M2 8 // kg
#define M1S 8.5 //kg
#define MF_INIT 0.5 //kg
#define G 9.81 // m/s2
#define THRUST 15 // N
#define FLOW_RATE 0.018 // kg/s

#define DIS_X  1.0
#define DIS_V  0

#define NNODES 10
#define PLAN_HORIZON 2.5  // seconds
#define CONTROL_HZ 10  // Be careful, adjust DELTA_T correspond to this parameter.
#define DELTA_T 0.1
#define KAPPA 0.0
#define USE_SIM_TIME true

using namespace PSOPT;

////////// Define Global Variables //////////
double position = 0.0;
double position_stamped;
double velocity = 0.0;
double velocity_stamped;
double FmassEst = MF_INIT;

MatrixXd trajH, trajV, trajT;  // matrices to store optimal trajectory

nav_msgs::Path optTraj;
std_msgs::Float32 control; // on-off control command
std_msgs::Float32 cont_control; //continuous control command
std_msgs::Float32 TD_cmd; // on-off command for down thruster
std_msgs::Float32 TU_cmd; // on-off command for up thruster



///////////////////////////////////////
////////// PSOPT Config Part //////////
///////////////////////////////////////

////// Define the end point cost function ///////

adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters, adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
    adouble HE = final_states[0] - (adouble)DIS_X;
    adouble VE = final_states[1] - (adouble)DIS_V;

    return 10*HE*HE + tf;
    //return tf;
}

///////// Define Lagrange cost function /////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
    adouble HE = states[0] - (adouble)DIS_X;
    adouble VE = states[1] - (adouble)DIS_V;
    return 20*HE*HE+ 0.5*(controls[0]*controls[0]+ controls[1]*controls[1]);
    //return 0;
}

/////// Define system dynamics ///////
void dae(adouble* derivatives, adouble* path, adouble* states, adouble* controls, 
         adouble* patameters, adouble& time, adouble* xad, int iphase, Workspace* workspace)
         {
            adouble xdot, vdot, mdot, fddot, fudot, fdout, fuout;

            adouble x = states[0];
            adouble v = states[1];
            adouble m = states[2];
            adouble fd = states[3];
            adouble fu = states[4];

            adouble u1 = controls[0]; // Downward thruster
            adouble u2 = controls[1]; // Upward thruster

            ////////// Thruster Dynamics //////////
            fddot = -27.7081*fd + 1.9349*u1;
            fdout = 215.9683*fd;

            fudot = -27.7081*fu + 1.9349*u2;
            fuout = 215.9683*fu;
            ///////////////////////////////////////
            xdot = v;
            vdot = (M2-M1S-m)*G/(M2+M1S+m) + fdout/(M2+M1S+m) 
                    - fuout/(M2+M1S+m);
            mdot = -FLOW_RATE*(u1+u2);

            derivatives[0] = xdot;
            derivatives[1] = vdot;
            derivatives[2] = mdot;
            derivatives[3] = fddot;
            derivatives[4] = fudot;                   
         }

////////// Define the event function //////////
void events(adouble* e, adouble* initial_states, adouble* final_states,
            adouble* parameters, adouble& t0, adouble& tf, adouble* xad,
            int iphase, Workspace* workspace)
{
    adouble x0 = initial_states[0];
    adouble v0 = initial_states[1];
    adouble m0 = initial_states[2];
    adouble fd0 = initial_states[3];
    adouble fu0 = initial_states[4];
    adouble xf = final_states[0];
    adouble vf = final_states[1];
    

    e[0] = x0;
    e[1] = v0;
    e[2] = m0;
    e[3] = xf;
    e[4] = vf;
    e[5] = fd0;
    e[6] = fu0;
    
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


///////// Contorl loop class definition /////////
class MainControlLoop
{
    public:
        MainControlLoop(ros::NodeHandle *nh,
            Prob &problemIn, Sol &solutionIn, Alg &algorithmIn)
        {
            last_control.resize(2,1);
            next_control.resize(2,1);
            rk4_control.resize(2,2);
            rk4_time.resize(1,2);
            rk4_states.resize(5,2);
            init_states.resize(5,1);
            ForceEst.resize(2,1);
            ForceEst<<0,0;
            Finterp_time.resize(1,1);
            FinterpD.resize(1,1);
            FinterpU.resize(1,1);

            problem = problemIn;
            solution = solutionIn;
            algorithm = algorithmIn;

            //Publish control (on-off)
            control_pub = nh->advertise<std_msgs::Float32>
                ("control", 1);

            // Publish the continuous control
            cont_control_pub = nh->advertise<std_msgs::Float32>
                ("cont_control",1);

            //! Publish the control of TDown and TUp
            TD_pub = nh->advertise<std_msgs::Float32>
                ("TD_cmd", 1);
            
            TU_pub = nh->advertise<std_msgs::Float32>
                ("TU_cmd", 1);
            
            // Timer initialization (duration 0.2 sec as default),
            // oneshoot = true, autostart = false
            timeToPub = nh->createTimer(ros::Duration(0.1), 
                &MainControlLoop::PubControlCb, this, true, false);

            

        }
        
        double curr_Time;
        adouble curr_Time_a;
        adouble& curr_Time_ref = curr_Time_a;
        double stamp_now, stamp_next;
        Prob problem;
        Sol solution;
        Alg algorithm;
             
        

        // vectors for rk4 propagate
        MatrixXd rk4_control, rk4_time, rk4_states, rk4_param, init_states;
        // For force interpolation
        MatrixXd Finterp_time, FinterpD,FinterpU, xStar_ForceD, xStar_ForceU, Uinterp;
        
        

        void GetNextControl(const ros::TimerEvent& event)
        {
            curr_Time = ros::Time::now().toSec(); //get current time.
            curr_Time_a = (adouble) curr_Time;
            curr_Time_ref = curr_Time_a;
            stamp_now = position_stamped; // define t_i
            stamp_next = stamp_now + DELTA_T; // define t_(i+1)
            

            rk4_control.col(0) = last_control.col(0); rk4_control.col(1) = last_control.col(0);
            rk4_time(0,0) = stamp_now; rk4_time(0,1) = stamp_next;
            init_states(0,0) = position;
            init_states(1,0) = velocity; 
            init_states(2,0) = FmassEst;
            printf("FORCE INITIAL \n");
            init_states(3,0) = ForceEst(0,0);
            init_states(4,0) = ForceEst(1,0);
            printf("INITIAL FINISHED! \n");
            
            // RK4 propagate to get next states extimate. rk4_states
            rk4_propagate(dae, rk4_control, rk4_time, 
                    init_states, rk4_param, problem, 1, rk4_states, NULL);

            // Generate initial guess states with RK4
            int nnodes = problem.phases(1).nodes(0);
            MatrixXd x0(5,nnodes);
            MatrixXd init_state_guess(5,1);
            init_state_guess(0,0) = rk4_states(0,1);
            init_state_guess(1,0) = rk4_states(1,1);
            init_state_guess(2,0) = rk4_states(2,1);
            init_state_guess(3,0) = rk4_states(3,1);
            init_state_guess(4,0) = rk4_states(4,1);
            MatrixXd& init_state_guess_ref = init_state_guess;

            problem.phases(1).guess.controls = ones(2,nnodes);
            problem.phases(1).guess.time = linspace(stamp_next, stamp_next+PLAN_HORIZON, nnodes);
            rk4_propagate(dae, problem.phases(1).guess.controls,
                problem.phases(1).guess.time, init_state_guess_ref, rk4_param,
                problem, 1, x0, NULL);
            
            problem.phases(1).guess.states = x0;

            ////////// problem bounds iinformation //////////
            problem.phases(1).bounds.lower.states << 0, -0.5, 0.0, 0.0, 0.0;
            problem.phases(1).bounds.upper.states << 3, 0.5, MF_INIT, 15, 15;

            problem.phases(1).bounds.lower.controls << 0.0, 0.0;
            problem.phases(1).bounds.upper.controls << 1.0, 1.0;

            problem.phases(1).bounds.lower.events 
                << position, velocity, FmassEst, DIS_X, DIS_V, 0,0;//ForceEst(0,0)-1,ForceEst(1,0)-1;
            problem.phases(1).bounds.upper.events
                 << position, velocity, FmassEst, DIS_X, DIS_V,0,0; //ForceEst(0,0)+1,ForceEst(1,0)+1;

            problem.phases(1).bounds.lower.StartTime = stamp_next; 
            problem.phases(1).bounds.upper.StartTime = stamp_next;

            problem.phases(1).bounds.lower.EndTime = stamp_next;
            problem.phases(1).bounds.upper.EndTime = stamp_next+PLAN_HORIZON;

            ////////// Call PSOPT to solve the problem //////////
            psopt(solution, problem, algorithm);
            //if(solution.error_flag) exit(0);

            ////////// Create control signal //////////
            MatrixXd xStar = solution.get_states_in_phase(1);
            MatrixXd uStar = solution.get_controls_in_phase(1);
            MatrixXd t_sol = solution.get_time_in_phase(1);
            MatrixXd lambda = solution.get_dual_costates_in_phase(1);
            ////////// Interpolate Force at next time stamp as forceEst //////////
            printf("FORCE INTERPOLATE!\n");
            Finterp_time << (stamp_next+DELTA_T);
            printf("START!\n");
            xStar_ForceD = xStar.block<1,NNODES>(3,0);  // a 1xNNODES matrix
            xStar_ForceU = xStar.block<1,NNODES>(4,0);  // a 1xNNODES matrix
            lagrange_interpolation(FinterpD, Finterp_time, t_sol, xStar_ForceD);
            lagrange_interpolation(FinterpU, Finterp_time, t_sol, xStar_ForceU);
            lagrange_interpolation(Uinterp, Finterp_time, t_sol, uStar);
            printf("INTERPOLATE FINISH! \n");
            ForceEst(0,0) = FinterpD(0,0);
            ForceEst(1,0) = FinterpU(0,0);
            ////////// Generate control signal from MP//////////
            printf("GENERATE CONTROL FROM MP \n");
            double mu_u1, mu_u2;
            MatrixXd TDown, TUp;
            TDown.resize(1,lambda.cols());
            TUp.resize(1,lambda.cols());
            for(int i = 0;i< lambda.cols(); i++)
            {
                mu_u1 = lambda(2,i)*FLOW_RATE + lambda(3,i)*7.854;
                mu_u2 = lambda(2,i)*FLOW_RATE + lambda(4,i)*7.854;    

                // For u1
                if(mu_u1 >0.25) TDown(0,i) = 1;
                else if(mu_u1 <= 0.25) TDown(0,i) = 0;
                else { printf("mu_u1 error!\n"); exit(0);};

                //FOR u2
                if (mu_u2 >0.4) TUp(0,i) = 1;
                else if(mu_u2 <=0.4) TUp(0,i) = 0;
                else{ printf("mu_u2 error! \n"); exit(0);};
            }
            //printf("PLOT CONTORL \n");
            //plot(t_sol,TDown,problem.name + ":TDown signal", "times(s)", "signal", "TDown");
            //plot(t_sol,TUp,problem.name + ":TUp signal", "times(s)", "signal", "TUp");            
            


            //Generate next control
            next_control(0,0) = TDown(0,0);
            next_control(1,0) = TUp(0,0);
            
            control.data = next_control(0,0) - next_control(1,0);

            // Push data into TD_cmd and TU_cmd, check if data is either 0 or 1 before
            // publish the data
            assert(TDown(0,0)==0 || TDown(0,0)==1);
            TD_cmd.data = TDown(0,0);
            assert(TUp(0,0)==0 || TUp(0,0)==1);
            TU_cmd.data = TUp(0,0);

            // create a timer to publish next_control at stamp_next.
            double time_now = ros::Time::now().toSec();
            timeToPub.setPeriod(ros::Duration(stamp_next-time_now), true);
            timeToPub.start(); // start the timer
                       
            last_control = next_control;





        }

        void PubControlCb(const ros::TimerEvent& event)
        {
        
            control_pub.publish(control);
            //cont_control_pub.publish(cont_control);
            TD_pub.publish(TD_cmd);
            TU_pub.publish(TU_cmd);
            timeToPub.stop();

        }

    private:
        ros::Publisher control_pub;
        ros::Publisher cont_control_pub;
        ros::Publisher TD_pub, TU_pub;
        ros::Timer timeToPub;
        MatrixXd last_control;
        MatrixXd next_control;
        MatrixXd ForceEst; // a column vector stores [TDown, TUp] estimate.
        


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
    problem.name = "1-D OCP";
    problem.outfilename = "1d.txt";

    //define problem level constants & do level 1 setup
    problem.nphases = 1;
    problem.nlinkages = 0;
    psopt_level1_setup(problem);

    // define phase related information & do level 2 setup
    problem.phases(1).nstates = 5;
    problem.phases(1).ncontrols = 2;
    problem.phases(1).nevents = 7;
    problem.phases(1).npath = 0;
    problem.phases(1).nodes <<NNODES;
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
    algorithm.nlp_iter_max = 50;
    algorithm.nlp_tolerance = 1e-3;

    algorithm.collocation_method = "Legendre";

    

    // Initialize the buffer size of trajectory
    // the trajectory contains 12 data points
    /*trajT.resize(1,12);
    trajH.resize(1,12);
    trajV.resize(1,12);*/

    ////////// ROS Setup //////////
    ros::init(argc, argv, "one_dim_ocp_MPver2_node");
    ros::NodeHandle nh;

    // Use Simulation Time Setting
    //nh.setParam("/use_sim_time", USE_SIM_TIME);

    // Create AsyncSpinner
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // subcribe height, velocity and mass estimate
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("height_est", 1, poseCb);

    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("velocity_est", 1, velocityCb);

    ros::Subscriber mass_sub = nh.subscribe<std_msgs::Float32>
        ("mass_est",1 , massCb);

    /*ros::Subscriber traj_sub = nh.subscribe<nav_msgs::Path>
        ("optTraj", 1, trajCb);*/

    
    // create an instance of MainControlLoop
    MainControlLoop main_control_loop(&nh, problem, solution, algorithm);

    ////////// Setup control loop timer ////
    ros::Timer controlPubTimer = nh.createTimer
        (ros::Duration(1.0/CONTROL_HZ),
        &MainControlLoop::GetNextControl, &main_control_loop);
    

    ros::waitForShutdown();   


    




    
}