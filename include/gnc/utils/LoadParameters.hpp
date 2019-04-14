#pragma once

#include <yaml-cpp/yaml.h>
#include <common/messages/ctrl_params_t.hpp>
#include <gnc/control/Controller.hpp>

namespace maav
{
namespace gnc
{
namespace utils
{
/*
 *      Reads vehicle and control params from yaml
 *      Calling function should generate the config file
 *      node for proper error handling
 */
maav::gnc::Controller::Parameters LoadParametersFromYAML(const YAML::Node& config_file);
}
}
}