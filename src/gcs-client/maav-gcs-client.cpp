#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>
#include "common/messages/MsgChannels.hpp"
#include "common/utils/GetOpt.hpp"
#include "common/utils/debug.hpp"
#include "gcs/Messages.hpp"

using zcm::ZCM;
using namespace YAML;
using namespace std;
using namespace maav;

class MessageForwarder
{
    zcm::ZCM& zcm;

public:
    explicit MessageForwarder(ZCM& zcm_in) : zcm{zcm_in} {}
    template <typename T>
    void forward_message(const zcm::ReceiveBuffer*, const string& channel, const T* msg)
    {
        zcm.publish(channel, msg);
    }
};

void toVehicle(ZCM& vehicle_zcm, ZCM& gcs_zcm)
{
    MAAV_DEBUG("Setting up communications with vehicle");
    MessageForwarder forwarder(std::ref(vehicle_zcm));
    gcs_zcm.subscribe(
        NAV_RUNSTATE_CMD, &MessageForwarder::forward_message<nav_runstate_t>, &forwarder);
    gcs_zcm.subscribe(PLANNER_CMD, &MessageForwarder::forward_message<waypoint_t>, &forwarder);
    gcs_zcm.subscribe(
        CAMERA_DISC_CMD, &MessageForwarder::forward_message<camera_disc_t>, &forwarder);
    gcs_zcm.subscribe(
        CTRL_PARAMS_CHANNEL, &MessageForwarder::forward_message<ctrl_params_t>, &forwarder);
    gcs_zcm.subscribe(IDLE_CHANNEL, &MessageForwarder::forward_message<idle_t>, &forwarder);
    gcs_zcm.subscribe(START_VISION, &MessageForwarder::forward_message<nav_runstate_t>, &forwarder);

    MAAV_DEBUG("Channels to vehicle:\n\t%s\n\t%s\n\t%s\n\t%s\n\t%s", NAV_RUNSTATE_CMD, PLANNER_CMD,
        CAMERA_DISC_CMD, "DJI", START_VISION);

    gcs_zcm.run();
}

void toGCS(ZCM& vehicle_zcm, ZCM& gcs_zcm)
{
    MAAV_DEBUG("Setting up communications with GCS");
    MessageForwarder forwarder(std::ref(gcs_zcm));
    vehicle_zcm.subscribe(
        NAV_RUNSTATE_STAT, &MessageForwarder::forward_message<nav_runstate_t>, &forwarder);
    vehicle_zcm.subscribe(
        CTRL_HEARTBEAT_CHANNEL, &MessageForwarder::forward_message<waypoint_t>, &forwarder);
    vehicle_zcm.subscribe(
        CAMERA_DISC_STAT, &MessageForwarder::forward_message<camera_disc_t>, &forwarder);
    vehicle_zcm.subscribe(
        VISION_STAT, &MessageForwarder::forward_message<nav_runstate_t>, &forwarder);
    vehicle_zcm.subscribe(PLANNER_STAT, &MessageForwarder::forward_message<waypoint_t>, &forwarder);
    vehicle_zcm.subscribe(STATE_CHANNEL, &MessageForwarder::forward_message<state_t>, &forwarder);
    vehicle_zcm.subscribe(
        OBST_HEARTBEAT_CHANNEL, &MessageForwarder::forward_message<nav_runstate_t>, &forwarder);

    MAAV_DEBUG("Channels to GCS:\n\t%s\n\t%s\n\t%s\n\t%s\n\t%s\n\t%s", NAV_RUNSTATE_STAT,
        CTRL_HEARTBEAT_CHANNEL, CAMERA_DISC_STAT, VISION_STAT, STATE_CHANNEL,
        OBST_HEARTBEAT_CHANNEL);

    vehicle_zcm.run();
}

int main(int argc, char** argv)
{
    GetOpt gopt;
    gopt.addBool('h', "help", false, "Display help text");
    gopt.addString(
        'c', "config", "../../config/gcs-client-config.yaml", "Location of YAML config file");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        cout << "Usage: " << argv[0] << " [options]" << endl;
        gopt.printHelp();
        return 1;
    }

    Node config = LoadFile(gopt.getString("config"));

    ZCM vehicle_zcm{config["Vehicle"]["url"].as<string>().c_str()};
    ZCM gcs_zcm{config["GCS"]["url"].as<string>().c_str()};

    if (not vehicle_zcm.good()) throw runtime_error{"Vehicle listener ZCM initialization failed!"};
    if (not gcs_zcm.good()) throw runtime_error{"GCS listener ZCM initialization failed!"};

    thread vt{toGCS, std::ref(vehicle_zcm), std::ref(gcs_zcm)};
    thread gt{toVehicle, std::ref(vehicle_zcm), std::ref(gcs_zcm)};

    vt.join();
    gt.join();

    return 0;
}
