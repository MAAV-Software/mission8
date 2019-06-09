#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/utils/GetOpt.hpp>
#include <gnc/measurements/Waypoint.hpp>
#include <common/messages/waypoint_t.hpp>

using YAML::Node;
using maav::gnc::Waypoint;
using std::cin;
using std::cout;
using std::string;
using std::stringstream;
using std::this_thread::sleep_for;
using std::to_string;
using std::vector;
using maav::GOAL_WAYPOINT_CHANNEL;
using namespace std::chrono;


int main(int argc, char** argv)
{
    // Command line options
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }
    zcm::ZCM zcm{"ipc"};
    string current_command;
    vector<string> wpt_in(4);
    while (cout << ">> " && getline(cin, current_command))
    {
        try
            {
                stringstream ss{current_command};
                waypoint_t wpt;
                int i = 0;
                while (i < 4 && ss >> wpt_in[i])
                {
                    wpt.pose[i] = std::stod(wpt_in[i]);
                    ++i;
                }
                if (i != 4 || ss >> wpt_in[3]) throw 1;

                zcm.publish(maav::GOAL_WAYPOINT_CHANNEL, &wpt);
            }
            catch (...)
            {
                if (!current_command.empty()) cout << "ERROR: Command not recognized\n";
            }
    }
}