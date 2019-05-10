// Realtime visualizer for Octomaps
#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <iostream>

#include <pangolin/pangolin.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <zcm/zcm-cpp.hpp>

#include <octovis/OcTreeDrawer.h>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/octomap_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include "vision/core/utilities.hpp"
#include "gnc/utils/ZcmConversion.hpp"


using std::atomic_bool;
using namespace std::chrono_literals;
using maav::OCCUPANCY_MAP_CHANNEL;
using maav::vision::zcmTypeToOctomap;
using octomap::OcTreeDrawer;
using std::cerr;

atomic_bool KILL = false;

void sigHandler(int);
void setSigHandlers();

int main(int argc, char** argv)
{
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/tools/visualizer-config.yaml", "Path to config.");
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

    ZCMHandler<octomap_t> map_handler;

    zcm.start();

    zcm.subscribe(OCCUPANCY_MAP_CHANNEL, &ZCMHandler<octomap_t>::recv, &map_handler);
    OcTreeDrawer drawer;

    while (!KILL)
    {
        if (map_handler.ready())
        {
            const auto msg = map_handler.msg();
            map_handler.pop();
            const std::shared_ptr<octomap::OcTree> tree = zcmTypeToOctomap(&msg);
            drawer.setOcTree(*tree);
            drawer.draw();

            std::this_thread::sleep_for(1s);
            cerr << "drew a map\n";
        }
    }
    zcm.stop();
}

void sigHandler(int) { KILL = true; }
void setSigHandlers()
{
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);
}

