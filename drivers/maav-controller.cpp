#include <atomic>
#include <csignal>
#include <iostream>
#include <string>

#include <common/utils/GetOpt.hpp>
#include <gnc/control/StateMachine.hpp>

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }

int main(int argc, char** argv)
{
    /*
     *      Signal handlers
     */
    signal(SIGINT, sig_handler);
    signal(SIGABRT, sig_handler);
    signal(SIGSEGV, sig_handler);
    signal(SIGTERM, sig_handler);

    /*
     *      Command line arguments
     */
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/gnc/control-config.yaml", "Path to control config");
    gopt.addBool('v', "verbose", true, "Print information");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    const std::string control_config_file = gopt.getString("config");
    /*
     *  Try to open the control config before passing it into
     *  the classes.  This alerts user to pesky cryptic error
     *  when invalid file is passed
     */
    try
    {
        YAML::LoadFile(control_config_file);
    }
    catch (...)
    {
        std::cout << "Could not find config file\nPlease provide command line option  \"-c  "
                     "<path-to-config>\"\n";
        return 2;
    }

    maav::gnc::StateMachine state_machine(control_config_file);
    state_machine.run(KILL);
}
