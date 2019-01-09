#include <string>
#include <utility>

#include <gnc/Constants.hpp>
#include <gnc/utils/LoadParameters.hpp>

using YAML::Node;
using std::make_pair;
using maav::gnc::Controller;
using maav::gnc::constants::DEG_TO_RAD;
namespace maav
{
namespace gnc
{
namespace utils
{
Controller::Parameters LoadParametersFromYAML(const YAML::Node& config_file)
{
    Controller::Parameters params;
    const YAML::Node& posGains = config_file["pid-gains"]["pos-ctrl"];
    params.pos_gains[0][0] = posGains["x"][0].as<double>();
    params.pos_gains[0][1] = posGains["x"][1].as<double>();
    params.pos_gains[0][2] = posGains["x"][2].as<double>();

    params.pos_gains[1][0] = posGains["y"][0].as<double>();
    params.pos_gains[1][1] = posGains["y"][1].as<double>();
    params.pos_gains[1][2] = posGains["y"][2].as<double>();

    params.pos_gains[2][0] = posGains["z"][0].as<double>();
    params.pos_gains[2][1] = posGains["z"][1].as<double>();
    params.pos_gains[2][2] = posGains["z"][2].as<double>();

    params.pos_gains[3][0] = posGains["yaw"][0].as<double>();
    params.pos_gains[3][1] = posGains["yaw"][1].as<double>();
    params.pos_gains[3][2] = posGains["yaw"][2].as<double>();

    const YAML::Node& rateGains = config_file["pid-gains"]["rate-ctrl"];
    params.rate_gains[0][0] = rateGains["x"][0].as<double>();
    params.rate_gains[0][1] = rateGains["x"][1].as<double>();
    params.rate_gains[0][2] = rateGains["x"][2].as<double>();

    params.rate_gains[1][0] = rateGains["y"][0].as<double>();
    params.rate_gains[1][1] = rateGains["y"][1].as<double>();
    params.rate_gains[1][2] = rateGains["y"][2].as<double>();

    params.rate_gains[2][0] = rateGains["z"][0].as<double>();
    params.rate_gains[2][1] = rateGains["z"][1].as<double>();
    params.rate_gains[2][2] = rateGains["z"][2].as<double>();

    params.rate_gains[3][0] = rateGains["ems-z"][0].as<double>();
    params.rate_gains[3][1] = rateGains["ems-z"][1].as<double>();
    params.rate_gains[3][2] = rateGains["ems-z"][2].as<double>();

    const Node& rateNode = config_file["limits"]["rate"];
    params.rate_limits[0] = make_pair(rateNode["x"][0].as<double>(), rateNode["x"][1].as<double>());
    params.rate_limits[1] = make_pair(rateNode["y"][0].as<double>(), rateNode["y"][1].as<double>());
    params.rate_limits[2] = make_pair(rateNode["z"][0].as<double>(), rateNode["z"][1].as<double>());
    params.rate_limits[3] =
        make_pair(rateNode["yaw"][0].as<double>(), rateNode["yaw"][1].as<double>());

    const Node& accelNode = config_file["limits"]["accel"];
    params.accel_limits[0] =
        make_pair(accelNode["x"][0].as<double>(), accelNode["x"][1].as<double>());
    params.accel_limits[1] =
        make_pair(accelNode["y"][0].as<double>(), accelNode["y"][1].as<double>());
    params.accel_limits[2] =
        make_pair(accelNode["z"][0].as<double>(), accelNode["z"][1].as<double>());

    const Node& angNode = config_file["limits"]["angle"];
    params.angle_limits[0] = make_pair(
        angNode["roll"][0].as<double>() * DEG_TO_RAD, angNode["roll"][1].as<double>() * DEG_TO_RAD);
    params.angle_limits[1] = make_pair(angNode["pitch"][0].as<double>() * DEG_TO_RAD,
        angNode["pitch"][1].as<double>() * DEG_TO_RAD);

    params.setpoint_tol = config_file["tol"].as<double>();
    params.takeoff_alt = config_file["takeoff-alt"].as<double>();

    params.zcm_url = config_file["zcm-url"].as<std::string>();
    params.ff_thrust = config_file["ff-thrust"].as<double>();

    return params;
}
}
}
}