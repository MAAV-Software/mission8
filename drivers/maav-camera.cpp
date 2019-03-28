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

#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/rgb_image_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <vision/core/CameraDriverHelper.hpp>

using std::atomic;
using std::condition_variable;
using std::mutex;
using std::string;
using std::unique_lock;
using std::unique_ptr;
using std::vector;
using zcm::ZCM;

using maav::vision::CameraDriverHelper;

// Used for synchronization with the kill signal
mutex mtx;
condition_variable cond_var;

// Assigns the lower serial number camera to be the bottom cam
// and assigns the higher serial number camera to be the forward cam

// Keeps track of whether the kill signal has been received
atomic<bool> KILL{false};

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
    gopt.addString('c', "config", "../config/camera-config.yaml", "Path to config.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));

    vector<unique_ptr<CameraDriverHelper>> helpers;
    helpers.reserve(3);
    // All cameras get the same camera pos channel
    // Since it is not intended for there to be more than one tracking camera
    // in use at one time. And also this way, each of the CameraDriverHelpers
    // can be configured to be the tracking camera in so desired.
    if (config["downward"]["enabled"].as<bool>())
        helpers.emplace_back(new CameraDriverHelper(config["downward"],
            CameraDriverHelper::FORMAT_IPC,
            maav::RGBD_DOWNWARD_CHANNEL, maav::DOWNWARD_CAMERA_POINT_CLOUD_CHANNEL,
            maav::CAMERA_POS_CHANNEL));
    if (config["forward"]["enabled"].as<bool>())
        helpers.emplace_back(new CameraDriverHelper(config["forward"],
            CameraDriverHelper::FORMAT_IPC,
            maav::RGBD_FORWARD_CHANNEL, maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL,
            maav::CAMERA_POS_CHANNEL));
    if (config["other-forward"]["enabled"].as<bool>())
        helpers.emplace_back(new CameraDriverHelper(config["other-forward"],
            CameraDriverHelper::FORMAT_IPC,
            maav::RGBD_FORWARD_CHANNEL, maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL,
            maav::CAMERA_POS_CHANNEL));

    for (unsigned i = 0; i < helpers.size(); ++i)
    {
        helpers[i]->beginRecording();
    }

    // Wait until the kill signal is received
    unique_lock<mutex> lck(mtx);
    while (!KILL)
    {
        cond_var.wait(lck);
    }
    // Helper destructor stops their operations so no need to
    // explicitly stop them
}
