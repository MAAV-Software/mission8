#include <iostream>

#include <zcm/zcm-cpp.hpp>
#include <memory>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/octomap_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include "gnc/Planner.hpp"
#include "vision/core/utilities.hpp"
#include "gnc/utils/ZcmConversion.hpp"
#include "gnc/State.hpp"
#include "gnc/planner/Path.hpp"

using maav::OCCUPANCY_MAP_CHANNEL;
using maav::STATE_CHANNEL;
using maav::PATH_CHANNEL;
using maav::GOAL_WAYPOINT_CHANNEL;
using maav::gnc::Planner;
using maav::vision::zcmTypeToOctomap;
using maav::gnc::State;

int main(int argc, char** argv)
{
    std::cout << "Guidance Driver" << std::endl;

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "", "Path to config.");
    // TODO: Add getopt arguments as necessary

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    zcm::ZCM zcm{"ipc"};
    zcm.start();

    ZCMHandler<octomap_t> map_handler;
    ZCMHandler<state_t> state_handler;
    ZCMHandler<waypoint_t> goal_handler;

    zcm.subscribe(OCCUPANCY_MAP_CHANNEL, &ZCMHandler<octomap_t>::recv, &map_handler);
    zcm.subscribe(STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
    zcm.subscribe(GOAL_WAYPOINT_CHANNEL, &ZCMHandler<waypoint_t>::recv, &goal_handler);

    Planner planner(gopt.getString("config"));

    bool kill = false;

    while (!kill)
    {
        if (map_handler.ready())
        {
            const auto msg = map_handler.msg();
            map_handler.pop();
            const std::shared_ptr<octomap::OcTree> tree = zcmTypeToOctomap(&msg);
            planner.update_map(tree);
        }

        if (state_handler.ready())
        {
            const auto msg = state_handler.msg();
            state_handler.pop();
            const State state = maav::gnc::ConvertState(msg);
            planner.update_state(state);
        }

        if (goal_handler.ready())
        {
            const auto msg = goal_handler.msg();
            const auto goal = maav::gnc::ConvertWaypoint(msg);
            planner.update_target(goal);
        }
        path_t path = maav::gnc::ConvertPath(planner.get_path());
        zcm.publish(PATH_CHANNEL, &path);
    }
    zcm.stop();
}