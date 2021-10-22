//////////////////////////////////////////////////////////////
//////////// 1-D Optimal control with ROS in MPC sheme////////
//////////////////////////////////////////////////////////////

#include "psopt.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

#define M2 9 // kg
#define M1S 8.5 //kg
#define MF_INIT 0.5 //kg
#define G 9.81 // m/s2
#define THRUST 15 // N
#define FLOW_RATE 0.018 // kg/s

#define DIS_X  1.5
#define DIS_V  0

using namespace PSOPT;

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
    adouble vf_error = final_states[1] = DIS_V;

    return 0.5*(xf_error*xf_error + vf_error*vf_error);


}

///////// Define Lagrange cost function /////////

adouble integrand_cost(adouble* states, adouble* controls, adouble* parameters,
                     adouble& time, adouble* xad, int iphase, Workspace* workspace)
{
    // TODO: Add control square cost
    adouble input = controls[0];

    return input*input;
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
    //TODO cancel final state constraint?;
}

void linkages( adouble* linkages, adouble* xad, Workspace* workspace)
{

}

/////////////////////////////////////
////////// ROS Config Part //////////
/////////////////////////////////////
double position = 0.0;
double position_stamped;
double velocity = 0.0;
double velocity_stamped;
double FmassEst = 0.5;

std_msgs::Float32 control;

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

////////////////////////////////
///////// Main Routine /////////
////////////////////////////////

int main(int argc, char **argv)
{
    ////////// ROS Setup //////////
    ros::init(argc, argv, "one_dim_ocp_node");
    ros::NodeHandle nh;

    // subcribe height, velocity and mass estimate
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("height", 1, poseCb);

    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("velocity", 1, velocityCb);

    ros::Subscriber mass_sub = nh.subscribe<std_msgs::Float32>
        ("mass_est",1 , massCb);

    //Publish control
    ros::Publisher control_pub = nh.advertise<std_msgs::Float32>
        ("control", 1);

    // set the control publish rate at 5Hz.
    ros::Rate rate(5);

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
    problem.phases(1).nevents = 3;
    problem.phases(1).npath = 0;
    problem.phases(1).nodes <<10;
    psopt_level2_setup(problem, algorithm);

    /*********** Move into ROS while loop **************
    // problem bounds information
    problem.phases(1).bounds.lower.states << 0, 0, 0;
    problem.phases(1).bounds.upper.states << 3, 2, MF_INIT;

    problem.phases(1).bounds.lower.controls << -1.0;
    problem.phases(1).bounds.upper.controls << 1.0;

    problem.phases(1).bounds.lower.events << 0, 0, MF_INIT;//, 1.4, 0;
    problem.phases(1).bounds.upper.events << 0, 0, MF_INIT;//, 1.6, 0;

    problem.phases(1).bounds.lower.StartTime = 0.0;
    problem.phases(1).bounds.upper.StartTime = 0.0;

    problem.phases(1).bounds.lower.EndTime = 0.0;
    problem.phases(1).bounds.upper.EndTime = 0.5;
    *******************************************************/

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
    float last_control = 0.0;
    ////////// The ROS While Loop //////////
    while(ros::ok())
    {
        // Get current time 
        double curr_Time = ros::Time::now().toSec();
        adouble curr_Time_a;
        curr_Time_a = (adouble) curr_Time;
        adouble& curr_Time_ref = curr_Time_a; // reference of current time.
        
        if (curr_Time < position_stamped)
        {
            printf("Time stamp error!");
            exit(0);
        }

        MatrixXd rk4_control(1,2);
        rk4_control(0,0) = last_control; rk4_control(0,1) = last_control;
        MatrixXd rk4_time(1,2);
        rk4_time(0,0) = position_stamped; rk4_time(0,1) = curr_Time;
        MatrixXd rk4_state(3,2);
        MatrixXd rk4_param;
        MatrixXd init_states(3,1);
        init_states(0,0) = position; init_states(1,0) = velocity; init_states(2,0) = FmassEst;
        // RK4 propagate from sample to current time
        rk4_propagate(&dae, rk4_control, rk4_time, init_states, rk4_param, problem,1,rk4_state, NULL);
                       

        // generate the propagation control and time sequence.
        // use last control signal as control trajectory.
        // the 

        
        ////////// Initial guess ////////// 
        int nnodes= problem.phases(1).nodes(0);

        // Use RK4 Propagation to generate initial guess.

        MatrixXd x0(3,nnodes);
        MatrixXd init_state_guess(3,1);
        init_state_guess(0,0) = rk4_state(0,1);
        init_state_guess(1,0) = rk4_state(1,1);
        init_state_guess(2,0) = rk4_state(2,1);

        MatrixXd& init_state_guess_ref = init_state_guess;

        problem.phases(1).guess.controls       = last_control*ones(1,nnodes);
        problem.phases(1).guess.time           = linspace(curr_Time, curr_Time+0.5, nnodes);
        rk4_propagate(&dae, problem.phases(1).guess.controls, problem.phases(1).guess.time,
                        init_state_guess_ref, rk4_param, problem, 1, x0, NULL);
        problem.phases(1).guess.states         = x0;
        // problem bounds information
        problem.phases(1).bounds.lower.states << 0, -2, 0;
        problem.phases(1).bounds.upper.states << 3, 2, MF_INIT;

        problem.phases(1).bounds.lower.controls << -1.0;
        problem.phases(1).bounds.upper.controls << 1.0;

        problem.phases(1).bounds.lower.events << position, velocity, FmassEst;//, 1.4, 0;
        problem.phases(1).bounds.upper.events << position, velocity, FmassEst;//, 1.6, 0;

        problem.phases(1).bounds.lower.StartTime = position_stamped; 
        problem.phases(1).bounds.upper.StartTime = position_stamped;

        problem.phases(1).bounds.lower.EndTime = position_stamped+2.5;
        problem.phases(1).bounds.upper.EndTime = position_stamped+2.5;

        ////////// Call PSOPT to solve the problem //////////
        psopt(solution, problem, algorithm);
        if (solution.error_flag) exit(0);

        ////////// Create control signal //////////
        MatrixXd uStar = solution.get_controls_in_phase(1);
        MatrixXd t_sol = solution.get_time_in_phase(1);
        // interpolate control at current time
        adouble control_now;
        //spline_interpolation(&control_now, curr_Time_ref, t_sol, uStar, nnodes);

        control_now = uStar(0);
        printf("control now value %f \n", control_now.value());

        // make a schidmit trigger to transform continuous command to on-off command
        float kappa = 0.1;

       if (last_control == 1)
       {
           if(control_now.value() >= (1-kappa)*0.5) control_now.setValue(1);
           else if(control_now.value() <= (1+kappa)*-0.5) control_now.setValue(-1);
           else control_now.setValue(0);
       }
       else if (last_control == 0)
       {
           if(control_now.value() >= (1+kappa)*0.5) control_now.setValue(1);
           else if(control_now.value() <= (1+kappa)*-0.5) control_now.setValue(-1);
           else control_now.setValue(0);
       }
       else if (last_control == -1)
       {
           if(control_now.value() >= (1+kappa)*0.5) control_now.setValue(1);
           else if(control_now.value() <= (1-kappa)*-0.5) control_now.setValue(-1);
           else control_now.setValue(0);
       }
       else{ printf("last control error!"); exit(0);}

        control.data = control_now.value();
        control_pub.publish(control);
        // store control value for next propagation
        last_control = control_now.value();

        ros::spinOnce();
        rate.sleep();      


    }




    
}