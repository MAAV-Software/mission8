#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <string>
#include <sstream>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/octomap_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <vision/core/utilities.hpp>

using std::atomic;
using std::condition_variable;
using std::mutex;
using std::string;
using std::unique_lock;
using std::unique_ptr;
using std::vector;
using std::stringstream;
using zcm::ZCM;
using std::shared_ptr;
using pcl::PointCloud;
using pcl::PointXYZ;
using std::chrono::milliseconds;
using std::thread;
using std::string;

using maav::vision::zcmTypeToOctomap;


/*
*  TO USE: The save octomap program must be run while the maav-octomap driver is running.
*  The saved .bt file can be used with the octovis program in thirdparty/octomap. 
*  Note that octovis will render the map upside down which is consistent with the MAAV coordinate system.
*  
*  You can either contiously update the map with "./maav-save-octomap"
*  Or you can usedadd the "-s" flag to grab one octomap from the MAP_CHANNEL and save that in a file
*  See the software/config/tools/save-octomap-config.yaml for the path the file is saved in.
*/

// Used for synchronization with the kill signal
mutex mtx;
condition_variable cond_var;
bool single_save = false;

// Keeps track of whether the kill signal has been received
atomic<bool> KILL{false};

class Handler
{
public:
    explicit Handler(YAML::Node& config) : path {config["save-path"].as<string>()} {}
    // Write the file to filename
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const octomap_t* message)
    {
        auto octomap = zcmTypeToOctomap(message);
        // Transform points from sensor to world
        string filename = path + std::to_string(message->utime) + ".ot";
        octomap->write(filename);

        if(single_save)
        {
            KILL = true;
            std::cerr << "Saved: " << filename << "\n";
            cond_var.notify_one();
        }
    }
private:
    string path;
};

void sigHandler(int)
{
    unique_lock<mutex> lck(mtx);
    KILL = true;
    cond_var.notify_one();
}

int main(int argc, char** argv)
{
    // Bind sigHandler
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/tools/save-octomap-config.yaml",
        "Path to config.");
    gopt.addBool('s', "single", false, "save a single octomap and quit");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    single_save = gopt.getBool("single");

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));
    // Start zcm, it handler processes every new point cloud
    zcm::ZCM zcm {"ipc"};
    Handler handler(config);
    zcm.subscribe(maav::OCCUPANCY_MAP_CHANNEL,
        &Handler::handle, &handler);
    zcm.start();

    // Wait until the kill signal is received
    unique_lock<mutex> lck(mtx);
    while (!KILL)
    {
        cond_var.wait(lck);
    }
    zcm.stop();
}
