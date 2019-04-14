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
#include <common/messages/control_commands_t.hpp>
#include <common/messages/localization_status_t.hpp>
#include <common/messages/path_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <gnc/measurements/Waypoint.hpp>

using YAML::Node;
using maav::gnc::Waypoint;
using std::cin;
using std::cout;
using std::string;
using std::stringstream;
using std::this_thread::sleep_for;
using std::to_string;
using std::vector;
using namespace std::chrono;

path_t create_test_path(const YAML::Node& path_file);

int main(int argc, char** argv)
{
    // Command line options
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('p', "test-path", "../config/gnc/test-path.yaml", "Read path to test path file");
    gopt.addString(
        'c', "config", "../config/gnc/control-config.yaml", "Control config for command zcm url");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    // Read path to path file;
    YAML::Node path_file;
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(gopt.getString("config"));
    }
    catch (...)
    {
        cout << "Could not find config file\nPlease provide command line option \"-c "
                "<path-to-config>\"\n";
        return 2;
    }

    zcm::ZCM zcm{config["command-zcm-url"].as<std::string>()};
    sleep_for(1s);

    control_commands_t commands;
    commands.takeoff = false;
    commands.land = false;
    commands.gains = false;
    localization_status_t localizer_status;
    localizer_status.localized = true;
    path_t path;
    path.utime = -1;
    path.NUM_WAYPOINTS = 0;

    // Spam some zcm messages, dumb
    zcm.publish(maav::PATH_CHANNEL, &path);
    zcm.publish(maav::CONTROL_COMMANDS_CHANNEL, &commands);
    zcm.publish(maav::LOCALIZATION_STATUS_CHANNEL, &localizer_status);

    string current_command;
    vector<string> wpt_in(4);
    while (cout << ">> " && getline(cin, current_command))
    {
        if (current_command == "path")
        {
            path_file = YAML::LoadFile(gopt.getString("test-path"));
            path = create_test_path(path_file);
            zcm.publish(maav::PATH_CHANNEL, &path);
            // try
            // {
            //     path_file = YAML::LoadFile(gopt.getString("test-path"));
            //     path = create_test_path(path_file);
            //     zcm.publish(maav::PATH_CHANNEL, &path);
            // }
            // catch (...)
            // {
            //     cout << "Path file not found\nProvide command line option \"-p
            //     <path-to-file>\"\n";
            //     continue;
            // }
        }

        else if (current_command == "clear")
        {
            if (system("clear")) cout << "Error\n";
        }

        else if (current_command == "takeoff")
        {
            commands.takeoff = true;
            zcm.publish(maav::CONTROL_COMMANDS_CHANNEL, &commands);
            commands.takeoff = false;
        }

        else if (current_command == "land")
        {
            commands.land = true;
            zcm.publish(maav::CONTROL_COMMANDS_CHANNEL, &commands);
            commands.land = false;
        }

        else if (current_command == "gains")
        {
            commands.gains = true;
            zcm.publish(maav::CONTROL_COMMANDS_CHANNEL, &commands);
            commands.gains = false;
        }

        else if (current_command == "lost")
        {
            localizer_status.localized = false;
        }

        else if (current_command == "found")
        {
            localizer_status.localized = true;
        }

        else if (current_command == "quit")
        {
            break;
        }

        else
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
                path_t wpt_path;
                wpt_path.NUM_WAYPOINTS = 1;
                wpt_path.waypoints.push_back(wpt);
                zcm.publish(maav::PATH_CHANNEL, &wpt_path);
            }
            catch (...)
            {
                if (!current_command.empty()) cout << "ERROR: Command not recognized\n";
            }
        }

        // For testing, TODO: move to localizer
        zcm.publish(maav::LOCALIZATION_STATUS_CHANNEL, &localizer_status);
    }
}

/*
 *	    Test path for testing tests
 *      Reads from yaml file (path can be provided)
 *      Make sure that the waypoints in yaml are named
 *      sequentially with integer values starting at 0
 */
// path_t create_test_path(const Node& path_file)
// {
//     path_t path;
//     waypoint_t wpt;
//     int waypoint_count = 0;
//     string waypoint_key;

//     const Node& path_node = path_file["path"];
//     for (auto it = path_node.begin(); it != path_file["path"].end(); ++it)
//     {
//         waypoint_key = to_string(waypoint_count);
//         wpt.pose[0] = path_node[waypoint_key][0].as<double>();
//         wpt.pose[1] = path_node[waypoint_key][1].as<double>();
//         wpt.pose[2] = path_node[waypoint_key][2].as<double>();
//         wpt.pose[3] = path_node[waypoint_key][3].as<double>();
//         path.waypoints.push_back(wpt);
//         ++waypoint_count;
//     }

//     path.NUM_WAYPOINTS = waypoint_count;

//     return path;
// }

path_t create_test_path(const Node& path_file)
{
    path_t path;
    waypoint_t wpt;
    int waypoint_count = 0;
    string waypoint_key;

    const Node& path_node = path_file["path"];
    for (auto it = path_node.begin(); it != path_file["path"].end(); ++it)
    {
        wpt.pose[0] = path_node[waypoint_count][0].as<double>();
        wpt.pose[1] = path_node[waypoint_count][1].as<double>();
        wpt.pose[2] = path_node[waypoint_count][2].as<double>();
        wpt.pose[3] = path_node[waypoint_count][3].as<double>();
        path.waypoints.push_back(wpt);
        ++waypoint_count;
    }

    path.NUM_WAYPOINTS = waypoint_count;

    return path;
}