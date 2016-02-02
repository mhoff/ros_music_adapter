#include <iostream>
#include <map>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <music.hh>
#include <mpi.h>

#include "sys/time.h"

#define DEBUG_OUTPUT false 

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_SENSOR_UPDATE_RATE = 30;
const std::string DEFAULT_ROS_NODE_NAME = "ros_sensor_node";

enum msg_types {Laserscan, Twist}; 

class RosSensorAdapter
{
    public:
        void init(int argc, char** argv);
        bool ratesMatch (double precision);
        void runMUSIC();
        void runROS();
	    void runROSMUSIC();
        void finalize();

    private:
        std::string ros_topic;
        ros::Subscriber subscriber;
        std::string ros_node_name;

        MPI::Intracomm comm;
	    MUSIC::Setup* setup;
        MUSIC::Runtime* runtime;
        double stoptime;
        int datasize;
        double sensor_update_rate;
        double timestep;

	    pthread_mutex_t data_mutex;
        double* data;

        msg_types msg_type;

        void initROS(int argc, char** argv);
        void initMUSIC(int argc, char** argv);

        void laserscanCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void twistCallback(const geometry_msgs::Twist msg);

};
