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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zcm/zcm-cpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/heartbeat_t.hpp>
#include <common/messages/octomap_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <gnc/OccupancyMap.hpp>
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

using maav::gnc::OccupancyMap;
using maav::vision::octomapToZcmType;
using namespace std::chrono;

// Used for synchronization with the kill signal
mutex mtx;
condition_variable cond_var;

// Keeps track of whether the kill signal has been received
atomic<bool> KILL{false};

void sigHandler(int)
{
    unique_lock<mutex> lck(mtx);
    KILL = true;
    cond_var.notify_one();
}
// Sends heartbeats every 100 milliseconds
// Also the main thread waits for this to end
// to know when the kill signal has been sent
void runHeartbeat(zcm::ZCM* zcm)
{
    heartbeat_t msg;
    msg.alive = true;
    while (!KILL)
    {
        zcm->publish(maav::OCCUPANCY_MAP_HEARTBEAT_CHANNEL, &msg);
        std::this_thread::sleep_for(milliseconds(100));
    }
}
// Sends saved map over zcm every 2 seconds
void publishMap(zcm::ZCM* zcm, const string &filename)
{
    AbstractOcTree* tree = AbstractOcTree::read(filename);
    OcTree* octree = dynamic_cast<OcTree*>(tree);
    octomap_t message;
    message.utime = last_update_;
    octomapToZcmType(octree, &message);
    while (!KILL)
    {
        milliseconds utime = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
        );
        message.utime = utime;
        zcm->publish(maav::OCCUPANCY_MAP_CHANNEL, &message);
        std::this_thread::sleep_for(milliseconds(2000));
    }
    delete octree;
}

int main(int argc, char** argv)
{
    // Bind sigHandler
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    if(argc != 2)
    {
        std::cerr << "USAGE: publish-load-map <filepath>"
        return -1;
    }

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    string filename = argv[1];

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));
    YAML::Node forward_camera = camera_config["forward"];
    // Start zcm, it handler processes every new point cloud
    zcm::ZCM zcm {"ipc"};
    zcm.start();
    // start threads
    thread heartbeat(runHeartbeat, &zcm);
    thread publishmap(publishMap, &zcm, filename)
    // Wait until the kill signal is received
    unique_lock<mutex> lck(mtx);
    while (!KILL)
    {
        cond_var.wait(lck);
    }
    zcm.stop();
    heartbeat.join();
    publishmap.join();
}
