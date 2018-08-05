#pragma once

#include "state.hpp"

namespace maav {
namespace gnc {

class Planner {
   public:
    Planner();

    void set_target();

    void add_state(const State& state);

   private:
};

}  // namespace gnc
}  // namespace maav
