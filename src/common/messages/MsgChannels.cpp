#include <common/messages/MsgChannels.hpp>

namespace maav
{
// clang-format off

// Camera driver messages
const char* const RGBD_FORWARD_CHANNEL = "FORWARD_RGBD";
const char* const RGBD_DOWNWARD_CHANNEL = "DOWNWARD_RGBD";
const char* const FORWARD_CAMERA_POINT_CLOUD_CHANNEL = "FORWARD_POINT_CLOUD";
const char* const DOWNWARD_CAMERA_POINT_CLOUD_CHANNEL = "DOWNWARD_POINT_CLOUD";

// GNC messages     
const char* const STATE_CHANNEL = "STATE";
const char* const IMU_CHANNEL = "IMU";
const char* const HEIGHT_LIDAR_CHANNEL = "HLIDAR";
const char* const PLANE_FIT_CHANNEL = "PLANE_FIT";
const char* const GLOBAL_UPDATE_CHANNEL = "GLOBAL_UPDATE";

const char* const SIM_STATE_CHANNEL = "SIM_STATE";
const char* const SIM_IMU_CHANNEL = "SIM_IMU";
const char* const SIM_HEIGHT_LIDAR_CHANNEL = "SIM_HLIDAR";
const char* const SIM_PLANE_FIT_CHANNEL = "SIM_PLANE_FIT";
const char* const SIM_GLOBAL_UPDATE_CHANNEL = "SIM_GLOBAL_UPDATE";

const char* const CTRL_PARAMS_CHANNEL = "CTRL_PARAMS";
const char* const MAP_CHANNEL = "MAP";
const char* const PATH_CHANNEL = "PATH";
const char* const SLAM_RESET_CHANNEL = "SLAM_RESET";
const char* const SLAM_LOCALIZATION_MODE_CHANNEL = "SLAM_LOC_MODE";

// Planefitter messages     
const char* const PLANE_FITTER_HEARTBEAT_CHANNEL = "PLANE_FITTER_HEARTBEAT";

const char* const VISUALIZER_CHANNEL = "VISUALIZER";

// Miscellaneous channels     
const char* const EMS_CHANNEL = "EMS";
const char* const PATH_PROGRESS_CHANNEL = "PATH_PROGRESS";
const char* const NAV_RUNSTATE_CMD_CHANNEL = "RUNSTATE_CMD";
const char* const NAV_RUNSTATE_STAT_CHANNEL = "RUNSTATE_STAT";
const char* const IDLE_CHANNEL = "IDLE";
const char* const CONTROL_COMMANDS_CHANNEL = "CONTROL";
const char* const KILLSWITCH_CHANNEL = "KILLSWITCH";
const char* const LOCALIZATION_STATUS_CHANNEL = "LOCALIZATION_STATUS";
const char* const PID_ERROR_CHANNEL = "PID_ERROR";
const char* const ATTITUDE_TARGET_CHANNEL = "ATTITUDE_TARGET";

// clang-format on
}  // namespace maav
