#include <cstdint>

namespace maav {
namespace gnc {
namespace constants {

/**
 * Nominal average acceleration from earths gravity
 */
constexpr double STANDARD_GRAVITY = 9.80665;

constexpr double SEC_TO_USEC = 1000000.0;
constexpr double USEC_TO_SEC = 1.0 / SEC_TO_USEC;

}  // namespace constants
}  // namespace gnc
}  // namespace maav
