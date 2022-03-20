#define _USE_MATH_DEFINES
#include <cmath>

#include "OneDimExpSystemModel.hpp"
#include "OneDimExpPositionMeasurementModle.hpp"

#include <ExtendedKalmanFilter.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> // Publish height estimate
#include <geometry_msgs/TwistStamped.h> // Publish velocity estimate
#include <std_msgs/Float32.h> // Publish fuel mass estimate
#include <sensor_msgs/Imu.h> // Subscribe imu data
#include <geometry_msgs/PointStamped.h> // Subcribe lidar data

#define OBSERVER_HZ  10
#define HEIGHT_TRIM 1 // in meter

using namespace OneDimKalman;

typedef double T;

// Some type shortcuts
typedef State<T> State;
typedef Input<T> Input;
typedef SystemModel<T> SystemModel;

typedef PositionMeasurement<T> PositionMeasurement;
typedef PositionMeasurementModel<T> PositionModel;
/**
 * @brief Global variables for storing data from callback functions
 * 
 * This global variables includes z-axis acceleration, height measurements and their
 * timestamps.
 * 
 */
double position = 0.0;
double position_stamped;
double Zacc = 0.0;
double Zacc_stamped;
double Thruster_Down;
double Thruster_Up;

geometry_msgs::PoseStamped H_est;
geometry_msgs::TwistStamped V_est;
std_msgs::Float32 M_est;


/**
 * @brief Kalman filter loop class
 * 
 * @note The imu data and lidar measurement are published at 100Hz and 20Hz, 
 *       the main observer loop is running at (OBSERVER_HZ), therefore the 
 *       ROS multi-threaded spining is needed.
 *       The class contains the definition of the publisher and the main
 *       Kalman Filter framework, the kalman filter will be called by a timer
 *       every (1/OBSERVER_HZ) seconds.
 * 
 */
class MainKalmanFilterLoop
{
    public:
        //! State variables
        State x;
        //! System inputs
        Input u;
        //! Define Kalman filter system model
        SystemModel sys;
        //! Define Kalman filter postion measurement model
        PositionModel pm;
        //! Extended Kalman Filter
        Kalman::ExtendedKalmanFilter<State> ekf;


        
        //! Define constructor
        MainKalmanFilterLoop(ros::NodeHandle *nh)
        {
            //! Define position, velocity and fuel mass publisher
            pose_pub = nh->advertise<geometry_msgs::PoseStamped>
                ("height_est", 1);
            twist_pub = nh->advertise<geometry_msgs::TwistStamped>
                ("velocity_est", 1);
            fuelmass_pub = nh->advertise<std_msgs::Float32>
                ("mass_est", 1);

            //! Define imu and lidar subscriber
            imu_sub = nh->subscribe<sensor_msgs::Imu>
                ("imu_raw", 1, imuCb);
            lidar_sub = nh->subscribe<geometry_msgs::PointStamped>
                ("height_measure", 1, lidarCb);
            TD_sub = nh->subscribe<std_msgs::Float32>("TD_cmd", 1, TDCb);
            TU_sub = nh->subscribe<std_msgs::Float32>("TU_cmd", 1, TUCb);
            
            x = State_Initialization();
            
            // Initialize it with current time.
            sys.setupSystemTime(ros::Time::now().toSec());
           
        }

        /**
         * @brief State initialization
         * 
         * Initialize the state first when start a Kalman filter.
         * @note initialized by hand. h=0, v=0, fm=0.5
         * //TODO initialized by first measurement
         * 
         * @returns An initialized state vector.
         */
        State State_Initialization()
        {
            State x_init;
            x_init.h() = 0;
            x_init.v() = 0;
            x_init.fm() = 0.5;

            return x_init;
        }

        /**
        * @brief IMU and Lidar callback functions
        * 
        * These functions are called by the ROS subscriber and store
        * data into the global variables.
        * The Kalman filter predict procedure will be done every time
        * when subscribe to new imu data and update at every lidar data
        * subscription.
        * 
        */
        void imuCb(const snesor_msgs::Imu::ConstPtr& msg)
        {
            Zacc = msg->linear_acceleration.z;
            Zacc_stamped = msg->header.stamp.toSec();

            // A Kalman filter predict
            // TODO how to prevent an incomming prediction while the update is not completed yet?
            // combine acc ane td, tu as input signal
            u.az() = Zacc;
            u.td() = Thruster_Down;
            u.tu() = Thruster_Up;

            // compute predict time interval
            sys.dt = Zacc_stamped - sys.timeStamp_now;

            // Update the timeStamp_now
            sys.timeStamp_now = Zacc_stamped;

            // State propagation
            x = ekf.predict(sys,u);
            

        }

        void lidarCb(const geometry_msgs::Pointstamped::ConstPtr& msg)
        {
            position = msg->point.z - HEIGHT_TRIM;
            position_stamped = msg-> header.stamp.toSec();

            //TODO Add a Kalman filter update
            // Predict the state to current time stamp
            sys.dt = position_stamped - sys.timeStamp_now;
            x = ekf.predict(sys,u);
            // updtate the  timestamp now
            sys.timeStamp_now = position_stamped;

            // Update EKF
            x = ekf.update(pm, position);
        }

        void TDCb(const std_msgs::Float32::ConstPtr& msg)
        {
            Thruster_Down = (double) msg->data;
        }

        void TUCb(const std_msgs::Float32::ConstPtr& msg)
        {
            Thruster_Up = (double) msg->data;
        }

        /**
         * @brief The callback function of the observer loop timer
         * 
         * This function will be called every observer loop to publish
         * the state estimation.
         * 
         */
        void PubStateEstimate()
        {
            // Packing the state
            // time stamp
            auto pubStamp = ros::Time::now().toSec();
            H_est.header.stamp = pubStamp;
            V_est.header.stamp = pubStamp;

            H_est.pose.position.x = x.h();
            V_est.twist.linear.x = x.v();
            M_est.data = (float) x.fm(); 

            //publish the data to topics
            pose_pub.publish(H_est);
            twist_pub.publish(V_est);
            fuelmass_pub.publish(M_est);
            
        }


    private:
        //! Declare the ros publishers and subscribers
        ros::Publisher pose_pub;
        ros::Publisher twist_pub;
        ros::Publisher fuelmass_pub;
        ros::Subscriber imu_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber TD_sub;
        ros::Subscriber TU_sub;

};  

int main(int argc, char **argv)
{
    // ROS setup
    ros::init(argc, argv, "one_dim_exp_ekf_node");
    ros::NodeHandle nh;

    // Create AsyncSpinner
    ros::AsyncSpinner spinner(0);
    spinner.start();

    MainKalmanFilterLoop main_ekf_loop(&nh);

    // Setup ekf loop timer
    ros::Timer ekfLoopTimer = nh.createTimer
        (ros::Duration(1/OBSERVER_HZ),
        &MainKalmanFilterLoop::PubStateEstimate, &main_ekf_loop);

    ros::waitForShutdown();
}