#include <atomic>
#include <csignal>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/attitude_target_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }
int main(int argc, char** argv)
{
    signal(SIGINT, sig_handler);
    signal(SIGABRT, sig_handler);
    signal(SIGSEGV, sig_handler);
    signal(SIGTERM, sig_handler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString(
        'c', "config", "../config/gnc/control-config.yaml", "Path to YAML control config");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(gopt.getString("config"));
    }
    catch (...)
    {
        std::cout << "Could not find config file\nPlease provide command line option \"-c "
                     "<path-to-config>\"\n";
        return 2;
    }

    zcm::ZCM zcm{config["zcm-url"].as<std::string>()};
    ZCMHandler<attitude_target_t> at_handler;
    zcm.subscribe(maav::ATTITUDE_TARGET_CHANNEL, &ZCMHandler<attitude_target_t>::recv, &at_handler);
    zcm.start();

    while (!KILL)
    {
        if (at_handler.ready())
        {
            std::cout << at_handler.msg().thrust << std::endl;
            at_handler.pop();
        }
    }
}