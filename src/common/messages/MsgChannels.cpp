#include <common/messages/MsgChannels.hpp>

namespace maav
{
// clang-format off

// Camera driver messages
const char* const RGBD_FORWARD_CHANNEL = "RGBD_FORWARD_CHANNEL";
const char* const RGBD_DOWNWARD_CHANNEL = "RGBD_DOWNWARD_CHANNEL";
const char* const FORWARD_CAMERA_POINT_CLOUD_CHANNEL = "FORWARD_POINT_CLOUD_CHANNEL";
const char* const DOWNWARD_CAMERA_POINT_CLOUD_CHANNEL = "DOWNWARD_POINT_CLOUD_CHANNEL";

// GNC messages     
const char* const STATE_CHANNEL = "STATE_CHANNEL";
const char* const IMU_CHANNEL = "IMU_CHANNEL";
const char* const HEIGHT_LIDAR_CHANNEL = "HLIDAR_CHANNEL";
const char* const PLANE_FIT_CHANNEL = "PLANE_FIT_CHANNEL";
const char* const GLOBAL_UPDATE_CHANNEL = "GLOBAL_UPDATE_CHANNEL";

const char* const SIM_STATE_CHANNEL = "SIM_STATE_CHANNEL";
const char* const SIM_IMU_CHANNEL = "SIM_IMU_CHANNEL";
const char* const SIM_HEIGHT_LIDAR_CHANNEL = "SIM_HLIDAR_CHANNEL";
const char* const SIM_PLANE_FIT_CHANNEL = "SIM_PLANE_FIT_CHANNEL";
const char* const SIM_GLOBAL_UPDATE_CHANNEL = "SIM_GLOBAL_UPDATE_CHANNEL";

const char* const CTRL_PARAMS_CHANNEL = "CTRL_PARAMS_CHANNEL";
const char* const MAP_CHANNEL = "MAP_CHANNEL";
const char* const PATH_CHANNEL = "PATH_CHANNEL";

// Planefitter messages     
const char* const PLANE_FITTER_HEARTBEAT_CHANNEL = "PLANE_FITTER_HEARTBEAT_CHANNEL";

// Miscellaneous channels     
const char* const EMS_CHANNEL = "EMS_CHANNEL";
const char* const PATH_PROGRESS_CHANNEL = "PATH_PROGRESS_CHANNEL";
const char* const NAV_RUNSTATE_CMD_CHANNEL = "RUNSTATE_CMD_CHANNEL";
const char* const NAV_RUNSTATE_STAT_CHANNEL = "RUNSTATE_STAT_CHANNEL";
const char* const IDLE_CHANNEL = "IDLE_CHANNEL";
const char* const CONTROL_COMMANDS_CHANNEL = "CONTROL_CHANNEL";
const char* const KILLSWITCH_CHANNEL = "KILLSWITCH_CHANNEL";
const char* const LOCALIZATION_STATUS_CHANNEL = "LOCALIZATION_STATUS_CHANNEL";

// clang-format on
}  // namespace maav
