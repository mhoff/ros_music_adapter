#include <iostream>
#include <map>
#include <math.h>

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>

#include <music.hh>
#include <mpi.h>

#include "sys/time.h"

#include <iostream>

#define DEBUG_OUTPUT false

const double DEFAULT_TIMESTEP = 1e-3;
const double DEFAULT_SENSOR_UPDATE_RATE = 100;
const double DEFAULT_RTF = 1.0;
const std::string DEFAULT_ROS_NODE_NAME = "music_dvs_adapter_node";

class DVSAdapter
{
public:
    void init(int argc, char** argv);
    bool ratesMatch(double precision);
    void runMUSIC();
    void runROS();
    void runROSMUSIC();
    void finalize();

private:
    std::string ros_topic;
    ros::Subscriber subscriber;
    std::string ros_node_name;
    double rtf;

    MPI::Intracomm comm;
    MUSIC::Setup* setup;
    MUSIC::Runtime* runtime;
    MUSIC::EventOutputPort* port_out;
    MUSIC::EventOutputPort* port_out_polarity[2];
    double stoptime;
    double sensor_update_rate;
    double timestep;
    double last_tick_time;

    void initROS(int argc, char** argv);
    void initMUSIC(int argc, char** argv);
    MUSIC::EventOutputPort* initOutput(std::string, int&);

    void eventArrayCallback(const dvs_msgs::EventArray msg);

};
