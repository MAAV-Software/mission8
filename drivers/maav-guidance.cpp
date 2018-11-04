#include <iostream>

#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/map_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/planner.hpp>

using maav::MAP_CHANNEL;
using maav::STATE_CHANNEL;
using maav::PATH_CHANNEL;
using maav::gnc::Planner;

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

    ZCMHandler<map_t> map_handler;
    ZCMHandler<state_t> state_handler;

    zcm.subscribe(MAP_CHANNEL, &ZCMHandler<map_t>::recv, &map_handler);
    zcm.subscribe(STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);

    Planner planner(gopt.getString("config"));

    bool kill = false;
    while (!kill)
    {
        if (map_handler.ready())
        {
            const auto msg = map_handler.msg();
            map_handler.pop();
            // TODO: Implement add map for planner
            //            planner.add_map(msg);
        }

        if (state_handler.ready())
        {
            const auto msg = state_handler.msg();
            state_handler.pop();

            // TODO: Implement add state for planner
            //            planner.add_state(msg);
        }

        // TODO: Get output from planner
        //        path_t path = localizer.get_path();
        //        zcm.publish(PATH_CHANNEL, &path);
    }
    zcm.stop();
}