#pragma once

#include "gnc/state.hpp"
//#include "gnc/Map.h"

namespace maav
{
namespace gnc
{
class Planner
{
   public:
    Planner(const std::string& path_config);

    // TODO: Add path/target
    void set_target();

    void add_state(const State& state);

    void add_map(/*const Map& map*/);

   private:
    std::string config_file;
};

}  // namespace gnc
}  // namespace maav
