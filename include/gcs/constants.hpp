#ifndef MAAV_GCS_CONSTANTS_HPP
#define MAAV_GCS_CONSTANTS_HPP

namespace maav
{
namespace gcs
{

/**
 * A good size for relatively large spaces in UI
 */
constexpr auto LARGE_SPACE = 10;

/**
 * A good size for normal spaces in UI
 */
constexpr auto MED_SPACE = 5;

/**
 * A good size for relatively small spaces in UI
 */
constexpr auto SMALL_SPACE = 2;

/**
 * Approximately how many seconds are required for the status state to change to
 * QUIET
 */
constexpr auto QUIET_TIMEOUT = 2;

/**
 * Approximately how many seconds are required for the status state to change to
 * DOWN (after being moved to QUIET)
 */
constexpr auto DOWN_TIMEOUT = 10;

}
}

#endif
