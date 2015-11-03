#include "connect.h"

int
main(int argc, char** argv)
{

    ConnectAdapter conn_adapter;
    conn_adapter.init(argc, argv);
    conn_adapter.runMUSIC();
    conn_adapter.finalize();

}

void
ConnectAdapter::init(int argc, char** argv)
{
    std::cout << "initializing connect adapter" << std::endl;
    timestep = DEFAULT_TIMESTEP;
    weights_filename = DEFAULT_WEIGHTS_FILENAME;
    initMUSIC(argc, argv);
    readWeightsFile();
}

void
ConnectAdapter::initMUSIC(int argc, char** argv)
{
    MUSIC::Setup* setup = new MUSIC::Setup (argc, argv);

    setup->config("stoptime", &stoptime);
    setup->config("music_timestep", &timestep);
    setup->config("weights_filename", &weights_filename);

    port_in = setup->publishContInput("in");
    port_out = setup->publishContOutput("out");

    comm = setup->communicator ();
    int rank = comm.Get_rank ();       
    int nProcesses = comm.Get_size (); 
    if (nProcesses > 1)
    {
        std::cout << "ERROR: num processes (np) not equal 1" << std::endl;
        comm.Abort(1);
    }

    // get dimensions of data
    if (port_in->hasWidth() && port_out->hasWidth())
    {
        size_data_in = port_in->width();
        size_data_out = port_out->width();
    }
    else
    {
        std::cout << "ERROR: Port-width not defined" << std::endl;
        comm.Abort(1);
    }
    
    data_in = new double[size_data_in];
    for (int i = 0; i < size_data_in; ++i)
    {
        data_in[i] = 0.;
    }

    data_out = new double[size_data_out];
    for (int i = 0; i < size_data_out; ++i)
    {
        data_out[i] = 0.;
    }

    weights = new double*[size_data_out];
    for (int i = 0; i < size_data_out; ++i)
    {
        weights[i] = new double[size_data_in];
    }


         
    // Declare where in memory to put command_data
    MUSIC::ArrayData dmap_in(data_in,
      		 MPI::DOUBLE,
      		 rank * size_data_in,
      		 size_data_in);
    port_in->map (&dmap_in, timestep, 1);
    
    MUSIC::ArrayData dmap_out(data_out,
      		 MPI::DOUBLE,
      		 rank * size_data_out,
      		 size_data_out);
    port_out ->map (&dmap_out, 1);

    runtime = new MUSIC::Runtime (setup, timestep);
}

void 
ConnectAdapter::readWeightsFile()
{
    Json::Reader json_reader;

    std::ifstream weights_file;
    weights_file.open(weights_filename.c_str(), std::ios::in);
    string json_weights_ = "";
    string line;

    while (std::getline(weights_file, line))
    {
        json_weights_ += line;
    }
    weights_file.close();
    
    if ( !json_reader.parse(json_weights_, json_weights))
    {
        // report to the user the failure and their locations in the document.
        std::cout   << "ERROR: linear readout: Failed to parse file \"" << weights_filename << "\"\n" 
                    << json_weights_ << " It has to be in JSON format.\n Using 1/N for each weight."
                    << json_reader.getFormattedErrorMessages();
        
        for (int i = 0; i < size_data_out; ++i)
        {
            for (int j = 0; j < size_data_in; ++j)
            {
                weights[i][j] = 1. / size_data_in;
            }
        }

        return;
    }
    else
    {
        for (int i = 0; i < size_data_out; ++i)
        {
            for (int j = 0; j < size_data_in; ++j)
            {
                weights[i][j] = json_weights[i][j].asDouble();
            }
        }

    }

}

void 
ConnectAdapter::runMUSIC()
{
    std::cout << "running connect adapter" << std::endl;
    
    Rate rate(1./timestep);
    struct timeval start;
    struct timeval end;
    gettimeofday(&start, NULL);
    unsigned int ticks_skipped = 0;

    double size_factor = double(size_data_in) / size_data_out;

    for (int t = 0; runtime->time() < stoptime; t++)
    {
        runtime->tick();
        for (int i = 0; i < size_data_out; ++i)
        {
            data_out[i] = 0;
            for (int j = i * size_factor; j < (i+1) * size_factor; ++j)
            {
                data_out[i] += data_in[j] * weights[i][j];
            }
        }

#if DEBUG_OUTPUT
        std::cout << "Connect Adapter: ";
        for (int i = 0; i < size_data_out; ++i)
        {
            std::cout << data_out[i] << " ";
        }
        std::cout << std::endl;
#endif

        rate.sleep();
    }

    gettimeofday(&end, NULL);
    unsigned int dt_s = end.tv_sec - start.tv_sec;
    unsigned int dt_us = end.tv_usec - start.tv_usec;
    if (end.tv_sec > start.tv_sec)
    {
        dt_us += 1000000;
    }
    std::cout << "connect adapter: total simtime: " << dt_s << " " << dt_us << " ticks skipped " << ticks_skipped <<  std::endl;
}

void
ConnectAdapter::finalize(){
    runtime->finalize();
    delete runtime;
}



