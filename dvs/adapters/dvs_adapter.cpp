#include "dvs_adapter.h"

#include "rtclock.h"
#include "dvs_msgs/Event.h"

#define BENCHMARK_ROS_CALLBACK 0
#define DEBUG_TICK 0
#define DEBUG_EVENT_TRANSFORMATION 0

#if BENCHMARK_ROS_CALLBACK
#include <time.h>
#endif

static void* ros_thread(void* arg)
{
    DVSAdapter* ros_adapter = static_cast<DVSAdapter*> (arg);
    ros_adapter->runROS();
}

int main(int argc, char** argv)
{

    DVSAdapter ros_adapter;
    ros_adapter.init(argc, argv);

    MPI::COMM_WORLD.Barrier();
    // If sensor_update_rate and timestep match to a relative
    // precision of 0.1%, lump the ROS and MUSIC event loops
    // together.
    if (ros_adapter.ratesMatch(0.001))
    {
        ros_adapter.runROSMUSIC();
    }
    else
    {
        pthread_t t;
        pthread_create(&t, NULL, ros_thread, &ros_adapter);

        ros_adapter.runMUSIC();
        pthread_join(t, NULL);
    }

    ros_adapter.finalize();
}

bool DVSAdapter::ratesMatch(double precision)
{
    return std::abs(sensor_update_rate * timestep - 1.) < precision;
}

void DVSAdapter::init(int argc, char** argv)
{
    std::cout << "initializing ROS dvs adapter" << std::endl;

    timestep = DEFAULT_TIMESTEP;
    sensor_update_rate = DEFAULT_SENSOR_UPDATE_RATE;
    ros_node_name = DEFAULT_ROS_NODE_NAME;
    rtf = DEFAULT_RTF;

    // MUSIC before ROS to read the config first!
    initMUSIC(argc, argv);
    initROS(argc, argv);
}

void DVSAdapter::initROS(int argc, char** argv)
{
    ros::init(argc, argv, ros_node_name);
    ros::start();

    ros::NodeHandle n;
    subscriber = n.subscribe(ros_topic, 1000, &DVSAdapter::eventArrayCallback, this);
}

void DVSAdapter::initMUSIC(int argc, char** argv)
{
    setup = new MUSIC::Setup(argc, argv);

    setup->config("ros_topic", &ros_topic);
    setup->config("stoptime", &stoptime);
    setup->config("sensor_update_rate", &sensor_update_rate);
    setup->config("ros_node_name", &ros_node_name);
    setup->config("rtf", &rtf);
    setup->config("music_timestep", &timestep);

    comm = setup->communicator();
    if (comm.Get_size() > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    int width_ = -1;

    port_out_polarity[0] = initOutput("on", width_);
    port_out_polarity[1] = initOutput("off", width_);
}

MUSIC::EventOutputPort* DVSAdapter::initOutput(std::string name, int &width)
{
    MUSIC::EventOutputPort* port = setup->publishEventOutput(name);
    if (port->isConnected())
    {
        if (not port->hasWidth())
        {
            std::cout << "ERROR: port " << name << " has no width" << std::endl;
            comm.Abort(2);
        }

        int width_ = port->width();
        if (width != -1 and width_ != width)
        {
            std::cout << "ERROR: node has inconsistent output widths (" << width << " != " << width_ << ")" << std::endl;
            comm.Abort(3);
        }
        width = width_;

        // map linear index to event out port
        MUSIC::LinearIndex l_index_out(0, width_);
        port->map(&l_index_out, MUSIC::Index::GLOBAL, 1);
    }
    else
    {
        port = NULL;
    }
    return port;
}

void DVSAdapter::runROSMUSIC()
{
    std::cout << "running sensor adapter with update rate of " << sensor_update_rate << std::endl;
    RTClock clock(1. / (sensor_update_rate * rtf));

    ros::spinOnce();
    runtime = new MUSIC::Runtime(setup, timestep);

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        last_tick_time = runtime->time();
        clock.sleepNext();
        ros::spinOnce();
        runtime->tick();
    }

    std::cout << "sensor: total simtime: " << clock.time() << " s" << std::endl;
}

void
DVSAdapter::runROS()
{
    RTClock clock(1. / (sensor_update_rate * rtf));

    // wait until first sensor update arrives
    while (ros::Time::now().toSec() == 0.)
    {
        clock.sleepNext();
    }

    ros::Time stop_time = ros::Time::now() + ros::Duration(stoptime / rtf);

    ros::spinOnce();
    for (ros::Time t = ros::Time::now(); t < stop_time; t = ros::Time::now())
    {
        clock.sleepNext();
        ros::spinOnce();
    }
}

void
DVSAdapter::runMUSIC()
{
    std::cout << "running dvs adapter with update rate of " << sensor_update_rate << std::endl;
    RTClock clock(timestep / rtf);

    runtime = new MUSIC::Runtime(setup, timestep);

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        last_tick_time = runtime->time();
        clock.sleepNext();
        runtime->tick();
    }

    std::cout << "sensor: total simtime: " << clock.time() << " s" << std::endl;
}

void
DVSAdapter::eventArrayCallback(const dvs_msgs::EventArray msg)
{
#if BENCHMARK_ROS_CALLBACK
    struct timespec start, finish;
    double elapsed;
    clock_gettime(CLOCK_MONOTONIC, &start);
#endif
    // msg.height * msg.width == port_out.getWidth()
#if DEBUG_EVENT_TRANSFORMATION
    if (msg.events.size() > 0)
    {
        std::cout << "eventArrayCallback " << msg.events.size() << std::endl;
    }
#endif
    // TODO verify delay
    double tick_time = last_tick_time + 2 * timestep;
    for (int i = 0; i < msg.events.size(); i++)
    {
        dvs_msgs::Event event = msg.events[i];
        int index = event.y * msg.width + event.x;
#if DEBUG_EVENT_TRANSFORMATION
        std::cout << "event: ts = " << tick_time << ", x = " << event.x << ", y = " << event.y << ", index = " << index << std::endl;
#endif
        /*
        if (port_out != NULL)
        {
            port_out->insertEvent(tick_time, MUSIC::GlobalIndex(index));
        }
        */
        MUSIC::EventOutputPort* port_pol = port_out_polarity[event.polarity];
        if (port_pol != NULL)
        {
            port_pol->insertEvent(tick_time, MUSIC::GlobalIndex(index));
        }
    }
#if BENCHMARK_ROS_CALLBACK
    clock_gettime(CLOCK_MONOTONIC, &finish);
    elapsed = (finish.tv_sec - start.tv_sec);
    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
    std::cout << "dvs adapter: elapsed time = " << elapsed << std::endl;
#endif
}

void DVSAdapter::finalize()
{
    runtime->finalize();
    delete runtime;
}
