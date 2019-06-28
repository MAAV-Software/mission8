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
void publishMap(zcm::ZCM* zcm, const string &filename, size_t between_publish_time)
{
    shared_ptr<const octomap::OcTree> octree((octomap::OcTree*)octomap::AbstractOcTree::read(filename));
    octomap_t message;
    octomapToZcmType(octree, &message);
    while (!KILL)
    {
        milliseconds utime = duration_cast< milliseconds >(
            system_clock::now().time_since_epoch()
        );
        message.utime = utime.count();
        zcm->publish(maav::OCCUPANCY_MAP_CHANNEL, &message);
        std::this_thread::sleep_for(milliseconds(between_publish_time));
    }
}

int main(int argc, char** argv)
{
    // Bind sigHandler
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    if(argc < 2)
    {
        std::cerr << "USAGE: publish-load-map <filepath> -w <time-between-publishes>\n";
        return -1;
    }

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addInt('w', "wait", "2000", "Time to wait between publishing maps.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        gopt.printHelp();
        return 1;
    }
    size_t wait_between = static_cast<size_t>(gopt.getInt("wait"));

    string filename = argv[1];


    // Start zcm, it handler processes every new point cloud
    zcm::ZCM zcm {"ipc"};
    // start threads
    thread heartbeat(runHeartbeat, &zcm);
    thread publishmap(publishMap, &zcm, filename, wait_between);
    // Wait until the kill signal is received
    unique_lock<mutex> lck(mtx);
    while (!KILL)
    {
        cond_var.wait(lck);
    }
    heartbeat.join();
    publishmap.join();
}