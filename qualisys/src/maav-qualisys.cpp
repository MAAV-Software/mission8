#include "QualisysZCM.hpp"

#include <atomic>
#include <csignal>

using namespace std;

atomic<bool> KILL{false};

void kill_handler(int) { KILL = true; }
int main(int argc, char** argv)
{
    // Install signal handling for SIGINT (Ctrl C) for graceful exit
    signal(SIGINT, kill_handler);

    // Determine config path
    string ypath = "../config/qualisys-config.yaml";
    if (argc > 1)
    {
        ypath = argv[1];
    }

    YAML::Node yconf;
    try
    {
        yconf = YAML::LoadFile(ypath);
    }
    catch (...)
    {
        cerr << "Could not find config file at " << ypath << endl;
        return -1;
    }

    qualisys::QualisysZCM qualisys_driver;
    if (!qualisys_driver.init(yconf))
    {
        cerr << "Initialization of the qualisys driver failed!" << endl;
        return -1;
    }

    while (!KILL)
    {
        qualisys_driver.run();
    }

    qualisys_driver.disconnect();

    return 0;
}
