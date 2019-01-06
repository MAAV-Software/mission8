#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>

#include <pangolin/pangolin.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include "FrameDrawer.hpp"
#include "MapDrawer.hpp"
#include "Viewer.hpp"

using std::atomic_bool;
using namespace std::chrono_literals;

using maav::tools::Viewer;
using maav::tools::MapDrawer;
using maav::tools::FrameDrawer;

atomic_bool KILL = false;

void sigHandler(int);
void setSigHandlers();

int main(int argc, char** argv)
{
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/imu-config.yaml", "Path to config.");
    gopt.addBool('v', "verbose", false, "Print readings.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    setSigHandlers();

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));

    zcm::ZCM zcm{config["zcm_url"].as<std::string>()};
    if (!zcm.good())
    {
        std::cerr << "Zcm bad" << std::endl;
        return 1;
    }

    // Construct and start viewer window
    bool display_images = config["display_images"].as<bool>();

    std::shared_ptr<FrameDrawer> frame_drawer(new FrameDrawer(config));
    if (!display_images) frame_drawer = nullptr;

    std::shared_ptr<MapDrawer> map_drawer(new MapDrawer(config));
    std::shared_ptr<Viewer> viewer(new Viewer(config, frame_drawer, map_drawer, zcm));

    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    std::this_thread::sleep_for(1s);

    zcm.start();

    while (!KILL)
    {
        std::this_thread::sleep_for(100ms);
    }

    zcm.stop();

    std::cout << "Stopping viewer thread." << std::endl;

    viewer->RequestFinish();
    viewer_thread.join();

    std::cout << "Thread stopped. Exiting." << std::endl;
}

void sigHandler(int) { KILL = true; }
void setSigHandlers()
{
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);
}
