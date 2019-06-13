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
#include <gnc/Constants.hpp>
#include <gnc/measurements/Waypoint.hpp>

using maav::gnc::Waypoint;
using std::cin;
using std::cout;
using std::string;
using std::stringstream;
using std::to_string;
using std::vector;
using std::this_thread::sleep_for;
using YAML::Node;
using namespace std::chrono;

void printHelp();

path_t create_figure_eight(double a);
path_t create_circle(double a);
path_t create_test_path(const YAML::Node& path_file);

constexpr double NUM_LOOPS = 5;
constexpr double DIAMETER = 5;

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
    commands.arm = false;
    commands.disarm = false;
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
        }

        else if (current_command == "fig8")
        {
            path_t fig_8_path = create_figure_eight(DIAMETER);
            zcm.publish(maav::PATH_CHANNEL, &fig_8_path);
        }

        else if (current_command == "circle")
        {
            path_t circle_path = create_circle(DIAMETER);
            zcm.publish(maav::PATH_CHANNEL, &circle_path);
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

        else if (current_command == "disarm")
        {
            commands.disarm = true;
            zcm.publish(maav::CONTROL_COMMANDS_CHANNEL, &commands);
            commands.disarm = false;
        }

        else if (current_command == "arm")
        {
            commands.arm = true;
            zcm.publish(maav::CONTROL_COMMANDS_CHANNEL, &commands);
            commands.arm = false;
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

        else if (current_command == "help")
        {
            printHelp();
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
                wpt.pose[3] *= maav::gnc::constants::DEG_TO_RAD;
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

path_t create_circle(double a)
{
    double dt = 0.05;
    path_t path;
    path.NUM_WAYPOINTS = 0;
    for (double t = 0; t <= NUM_LOOPS * 2 * M_PI; t += dt)
    {
        const double x = a * std::sin(t);
        const double y = a * std::cos(t);

        const double dx = a * std::cos(t);
        const double dy = -a * std::sin(t);
        Eigen::Vector2d velocity{dx, dy};
        velocity.normalize();

        const double yaw = std::atan2(velocity.y(), velocity.x());

        constexpr double altitude = -1;

        waypoint_t waypoint;
        waypoint.pose[0] = x;
        waypoint.pose[1] = y;
        waypoint.pose[2] = altitude;
        waypoint.pose[3] = yaw;
        path.waypoints.push_back(waypoint);
        path.NUM_WAYPOINTS++;
    }

    return path;
}

path_t create_figure_eight(double a)
{
    double dt = 0.05;
    path_t path;
    path.NUM_WAYPOINTS = 0;
    for (double t = 0; t <= NUM_LOOPS * 2 * M_PI; t += dt)
    {
        const double x = a * std::sin(t);
        const double y = a * std::sin(t) * std::cos(t);

        const double dx = a * std::cos(t);
        const double dy = -a * (std::cos(t) * std::cos(t) - std::sin(t) * std::sin(t));
        Eigen::Vector2d velocity{dx, dy};
        velocity.normalize();

        const double yaw = -std::atan2(velocity.y(), velocity.x());

        constexpr double altitude = -1;

        waypoint_t waypoint;
        waypoint.pose[0] = x;
        waypoint.pose[1] = y;
        waypoint.pose[2] = altitude;
        waypoint.pose[3] = yaw;
        path.waypoints.push_back(waypoint);
        path.NUM_WAYPOINTS++;
    }

    return path;
}

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

void printHelp()
{
    // clang-format off
    std::cout << std::endl;
    std::cout << "============================================================================"        << std::endl;
    std::cout << "X Y Z YAW - Moves quad to this position and heading. Yaw is in degrees."             << std::endl;
    std::cout << "help      - This message."                                                           << std::endl;
    std::cout << "quit      - Quits program."                                                          << std::endl;
    std::cout << "takeoff   - Takes off to configured takeoff altitude."                               << std::endl;
    std::cout << "land      - Descends and lands wherever the quad is currently."                      << std::endl;
    std::cout << "fig8      - Flys in a predefined figure 8 path."                                     << std::endl;
    std::cout << "circle    - Flys in a predefined circular path."                                     << std::endl;
    std::cout << "gains     - Uploads the gains from the config file. Gains can be set in mid flight." << std::endl;
    std::cout << "arm       - Arms the quad, but doesn't take off. Can only be done on the ground."    << std::endl;
    std::cout << "disarm    - Disarms quad. Can only be done when on the ground."                      << std::endl;
    std::cout << "============================================================================"        << std::endl;
    std::cout << std::endl;
    // clang-format on
}
