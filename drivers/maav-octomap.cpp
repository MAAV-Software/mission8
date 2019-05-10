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
#include <common/messages/rgb_image_t.hpp>
#include <common/messages/point_cloud_t.hpp>
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
using maav::gnc::PointMapper;
using maav::vision::zcmTypeToPCLPointCloud;
using maav::vision::octomapToZcmType;

// Used for synchronization with the kill signal
mutex mtx;
condition_variable cond_var;

class Handler
{
public:
    Handler(YAML::Node& config, YAML::Node& camera_config,
        zcm::ZCM &zcm) : occupancyMap_(config, camera_config, zcm),
        zcm_ {zcm} {}
    // Updates job dispatcher with new task data
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const point_cloud_t* message)
    {
        unique_lock<mutex> lck(mtx_);
        // If not currently processing a point cloud, process one
        // Since processing one takes a long time, this prevents large
        // backups and keeps the map current
        if (!currently_working_)
        {
                currently_working_ = true;
                PointCloud<PointXYZ>::Ptr cloud = zcmTypeToPCLPointCloud(*message);
                thread processing(process_cloud, cloud, message->utime, this);
                processing.detach();
        }
    }
    // Runs in a detached thread so that messages that can't be processed
    // can be discarded rather than back up
    static void process_cloud(PointCloud<PointXYZ>::Ptr cloud, uint64_t utime,
        Handler* handler)
    {
        // Update occupancy map with cloud measurements
        handler->occupancyMap_.update(cloud, utime);
        handler->last_update_ = utime;
        handler->sendMap();
        unique_lock<mutex> lck(handler->mtx_);
        handler->currently_working_ = false;
    }
    shared_ptr<const octomap::OcTree> getMap() {return occupancyMap_.map();}
    // Serialize the octomap and send it over zcm
    void sendMap()
    {
        octomap_t message;
        auto octmap = getMap();
        message.utime = last_update_;
        octomapToZcmType(octmap, &message);
        zcm_.publish(maav::OCCUPANCY_MAP_CHANNEL, &message);
    }
    void kill() {occupancyMap_.kill();}
private:
    mutex mtx_;
    bool currently_working_ = false;
    OccupancyMap occupancyMap_;
    zcm::ZCM& zcm_;
    long long last_update_ = 0;
};

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

int main(int argc, char** argv)
{
    // Bind sigHandler
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/gnc/octomap-config.yaml",
        "Path to config.");
    gopt.addString('k', "camera-config", "../config/camera-config.yaml",
        "Path to camera config.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));
    YAML::Node camera_config = YAML::LoadFile(gopt.getString("camera-config"));
    YAML::Node forward_camera = camera_config["forward"];
    // Start zcm, it handler processes every new point cloud
    zcm::ZCM zcm {"ipc"};
    Handler handler(config, forward_camera, zcm);
    zcm.subscribe(maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL,
        &Handler::handle, &handler);
    zcm.start();
    // start heartbeat thread
    thread heartbeat(runHeartbeat, &zcm);

    // Wait until the kill signal is received
    unique_lock<mutex> lck(mtx);
    while (!KILL)
    {
        cond_var.wait(lck);
    }
    handler.kill();
    zcm.stop();
    heartbeat.join();
}
