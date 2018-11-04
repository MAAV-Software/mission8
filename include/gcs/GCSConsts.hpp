#ifndef MAAV_GCS_CONSTANTS_HPP
#define MAAV_GCS_CONSTANTS_HPP
#include <yaml-cpp/yaml.h>
#include <array>
#include <string>

namespace maav
{
namespace gcs
{
/**
* @brief A container for holding constants for GCS
*/
class GCSConsts
{
    public:
    // Spacing for GUI
    const int SMALL_SPACE;
    const int MED_SPACE;
    const int LARGE_SPACE;

    // Timeout times (in seconds) for Status Frame
    const int QUIET_TIMEOUT;
    const int DOWN_TIMEOUT;

    // Default POS values loaded to Tuning Frame
    const double POS_X[3];
    const double POS_Y[3];
    const double POS_Z[3];
    const double POS_YAW[3];

    // Deafult RATE values loaded to Tuning Frame
    const double RATE_X[3];
    const double RATE_Y[3];
    const double RATE_Z[3];

    // Config file used to save gains
    const std::string FILENAME;

    // ZCM url
    const std::string URL;

    /**
    * @brief Writes contents of this GCSConsts aling
    * with given gains to FILENAME in YAML format.
    * This should be used to save gains in GCS.
    *
    * @param gains Gains to write.
    */
    void writeGains(std::array<double, 21> gains) const;

};  // class GCSConsts
}  // namespace gcs
}  // namespace maav
#endif
