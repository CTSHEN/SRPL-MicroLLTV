/*************************************************************
* 1-D Optimal control v0.2 
* v0.3:
* * Add x1_error into the integral cost.
* v0.4 : This is the program for controller
* A planner-controller structure.
* A planner generates a minimum-time trajectory and updates per second.
* The controller will track the optimal trajectory by minimizing the tracking error.

*************************************************************/

#include "psopt.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

#define M2 8 // kg
#define M1S 8.5 //kg
#define MF_INIT 0.5 //kg
#define G 9.81 // m/s2
#define THRUST 15 // N
#define FLOW_RATE 0.018 // kg/s

#define DIS_X  1.5//0.3
#define DIS_V  0

#define DELTA_T 0.2
#define KAPPA 0.0
#define USE_SIM_TIME true

using namespace PSOPT;

////////// Define Global Variables //////////
double position = 0.0;
double position_stamped;
double velocity = 0.0;
double velocity_stamped;
double FmassEst = MF_INIT;

std_msgs::Float32 control; // on-off control command
std_msgs::Float32 cont_control; //continuous control command
/********** DEBUG INFORMATION ***************/
std_msgs::Float32 timeStampToPub; // store and pub the value of stamp_next
std_msgs::Float32 pubTimeStamp; // True timestamp when the control msg was published.
/********************************************/


///////////////////////////////////////
////////// PSOPT Config Part //////////
///////////////////////////////////////

////// Define the end point cost function ///////

adouble endpoint_cost(adouble* initial_states, adouble* final_states,
                      adouble* parameters, adouble& t0, adouble& tf,
                      adouble* xad, int iphase, Workspace* workspace)
{
    // TODO: add end point error cost
    adouble xf_error = final_states[0] - DIS_X;
    adouble vf_error = final_states[1] - DIS_V;

    return (xf_error*xf_error + 0.1*vf_error*vf_error);
    //return tf;


}

///////// Define Lagrange cost function /////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
    // TODO: Add control square cost
    adouble input = controls[0];
    adouble x1_error = states[0] - DIS_X;
    adouble v1_error = states[1] - DIS_V;

    return 0.5*input*input +(x1_error*x1_error);
    //return 0;
}

/////// Define system dynamics ///////
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
    //adouble xf = final_states[0];
    //adouble vf = final_states[1];
    

    e[0] = x0;
    e[1] = v0;
    e[2] = m0;
    //e[3] = xf;
    //e[4] = vf;
    
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
            last_control = 0.0;
            rk4_control.resize(1,2);
            rk4_time.resize(1,2);
            rk4_states.resize(3,2);
            init_states.resize(3,1);

            problem = problemIn;
            solution = solutionIn;
            algorithm = algorithmIn;

            //Publish control (on-off)
            control_pub = nh->advertise<std_msgs::Float32>
                ("control", 1);

            // Publish the continuous control
            cont_control_pub = nh->advertise<std_msgs::Float32>
                ("cont_control",1);

            /**************** DEBUG INFORMATION****************/
            // Publish the calculated stamp_next
            calc_nextStamp_pub = nh -> advertise<std_msgs::Float32>
                ("calc_nextStamp",1);
            // Actual pub stamp
            actual_stamp_pub = nh -> advertise<std_msgs::Float32>
                ("actual_pub_stamp",1); 
            /*************************************************/


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
        
        

        void GetNextControl(const ros::TimerEvent& event)
        {
            curr_Time = ros::Time::now().toSec(); //get current time.
            curr_Time_a = (adouble) curr_Time;
            curr_Time_ref = curr_Time_a;
            stamp_now = position_stamped; // define t_i
            stamp_next = stamp_now + DELTA_T; // define t_(i+1)
            /*************** DEBUG INFORMATION ****************/
            timeStampToPub.data = stamp_next;
            /*************************************************/

            rk4_control(0,0) = last_control; rk4_control(0,1) = last_control;
            rk4_time(0,0) = stamp_now; rk4_time(0,1) = stamp_next;
            init_states(0,0) = position;
            init_states(1,0) = velocity; 
            init_states(2,0) = FmassEst;
            
            // RK4 propagate to get next states extimate. rk4_states
            rk4_propagate(dae, rk4_control, rk4_time, 
                    init_states, rk4_param, problem, 1, rk4_states, NULL);

            // Generate initial guess states with RK4
            int nnodes = problem.phases(1).nodes(0);
            MatrixXd x0(3,nnodes);
            MatrixXd init_state_guess(3,1);
            init_state_guess(0,0) = rk4_states(0,1);
            init_state_guess(1,0) = rk4_states(1,1);
            init_state_guess(2,0) = rk4_states(2,1);
            MatrixXd& init_state_guess_ref = init_state_guess;

            problem.phases(1).guess.controls = last_control*ones(1,nnodes);
            problem.phases(1).guess.time = linspace(stamp_next, stamp_next+2.5, nnodes);
            rk4_propagate(dae, problem.phases(1).guess.controls,
                problem.phases(1).guess.time, init_state_guess_ref, rk4_param,
                problem, 1, x0, NULL);
            
            problem.phases(1).guess.states = x0;

            ////////// problem bounds iinformation //////////
            problem.phases(1).bounds.lower.states << 0, -2, 0;
            problem.phases(1).bounds.upper.states << 3, 2, MF_INIT;

            problem.phases(1).bounds.lower.controls << -1.0;
            problem.phases(1).bounds.upper.controls << 1.0;

            problem.phases(1).bounds.lower.events << position, velocity, FmassEst;//, 0.7, 0;
            problem.phases(1).bounds.upper.events << position, velocity, FmassEst;//, 0.7, 0;

            problem.phases(1).bounds.lower.StartTime = stamp_next; 
            problem.phases(1).bounds.upper.StartTime = stamp_next;

            problem.phases(1).bounds.lower.EndTime = stamp_next+2.5;
            problem.phases(1).bounds.upper.EndTime = stamp_next+2.5;

            ////////// Call PSOPT to solve the problem //////////
            psopt(solution, problem, algorithm);
            if(solution.error_flag) exit(0);

            ////////// Create control signal //////////
            MatrixXd xStar = solution.get_states_in_phase(1);
            MatrixXd uStar = solution.get_controls_in_phase(1);
            MatrixXd t_sol = solution.get_time_in_phase(1);
            next_control = uStar(0);
            cont_control.data = next_control.value();

            // make a schidmit trigger to transform continuous command to on-off command
        

            if (last_control == 1)
            {
                if(next_control.value() >= (1-KAPPA)*0.5) next_control.setValue(1);
                else if(next_control.value() <= (1+KAPPA)*-0.5) next_control.setValue(-1);
                else next_control.setValue(0);
            }
            else if (last_control == 0)
            {
                if(next_control.value() >= (1+KAPPA)*0.5) next_control.setValue(1);
                else if(next_control.value() <= (1+KAPPA)*-0.5) next_control.setValue(-1);
                else next_control.setValue(0);
            }
            else if (last_control == -1)
            {
                if(next_control.value() >= (1+KAPPA)*0.5) next_control.setValue(1);
                else if(next_control.value() <= (1-KAPPA)*-0.5) next_control.setValue(-1);
                else next_control.setValue(0);
            }
            else{ printf("last control error!"); exit(0);}

            control.data = next_control.value();

            // create a timer to publish next_control at stamp_next.
            double time_now = ros::Time::now().toSec();
            timeToPub.setPeriod(ros::Duration(stamp_next-time_now), true);
            timeToPub.start(); // start the timer
            /****************** DEBUG INFORMATION ***********************/
            printf("stamp_last = %f \n", stamp_now);
            printf("time now = %f \n", time_now);
            printf("stamp_next = %f\n", stamp_next);

            //plot(t_sol,xStar,problem.name + ":states", "times(s)", "states", "x v m");
            //plot(t_sol,uStar,problem.name + ": control", "time (s)", "control", "u");
            /**********************************************************/
            
            last_control = next_control.value();





        }

        void PubControlCb(const ros::TimerEvent& event)
        {
            pubTimeStamp.data = ros::Time::now().toSec();
            control_pub.publish(control);
            cont_control_pub.publish(cont_control);
            /**************DEBUG INFORMATION *********************/
            calc_nextStamp_pub.publish(timeStampToPub);
            actual_stamp_pub.publish(pubTimeStamp);
            printf("actual publish time = %f", ros::Time::now().toSec());
            /****************************************************/
            timeToPub.stop();

        }

    private:
       ros::Publisher control_pub;
       ros::Publisher cont_control_pub;
       /*****************8DEBUG INFORMATION ***************/
       ros::Publisher calc_nextStamp_pub, actual_stamp_pub;
       /***************************************************/
       ros::Timer timeToPub;
        float last_control;
        adouble next_control;
        


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
    problem.phases(1).nstates = 3;
    problem.phases(1).ncontrols = 1;
    problem.phases(1).nevents = 3;//5;
    problem.phases(1).npath = 0;
    problem.phases(1).nodes <<10;
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
    ros::init(argc, argv, "one_dim_ocp_node");
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

    
    // create an instance of MainControlLoop
    MainControlLoop main_control_loop(&nh, problem, solution, algorithm);

    ////////// Setup control loop timer ////
    ros::Timer controlPubTimer = nh.createTimer
        (ros::Duration(1.0/5.0),
        &MainControlLoop::GetNextControl, &main_control_loop);    
    

    ros::waitForShutdown();   


    




    
}