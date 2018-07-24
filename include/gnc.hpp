#ifndef GNC_HPP
#define GNC_HPP

#include <iostream>

namespace maav {
namespace gnc {

class Localizer {
   public:
    Localizer();

    bool is_cool();
};

}  // namespace gnc
}  // namespace maav

#endif