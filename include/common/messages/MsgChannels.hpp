#ifndef MSGCHANNELS_HPP
#define MSGCHANNELS_HPP

namespace maav
{
extern const char* const CTRL_CHANNEL;          ///< controller output to inner loop through tanfan
extern const char* const CTRL_PARAMS_CHANNEL;   ///< PID gains from GCS client to controller
extern const char* const STATE_CHANNEL;         ///< state output from localization to system
extern const char* const SIM_STATE_CHANNEL;     ///< state output from sim to system
extern const char* const MAP_CHANNEL;           ///< map output from localization to system
extern const char* const OBSTS_CHANNEL;         ///< localization's obstacle output to system
extern const char* const IMU_CHANNEL;           ///< imu channel from tanfan to system
extern const char* const SIM_TRUE_IMU_CHANNEL;  ///< "true" imu readings from simulator
extern const char* const HEIGHT_LIDAR_CHANNEL;  ///< height lidar's output from tanfan to system
extern const char* const
    SIM_TRUE_HEIGHT_LIDAR_CHANNEL;          ///< "true" height lidar readings from simulator
extern const char* const SIM_TIME_CHANNEL;  ///< simulator time updates
extern const char* const EMS_CHANNEL;       ///< Tiva's emergency message from tanfan to system
extern const char* const PATH_CHANNEL;      ///< path output from mission planner to system
extern const char* const
    PATH_PROGRESS_CHANNEL;  ///< The outer loop controller's progress through its path
extern const char* const CTRL_HEARTBEAT_CHANNEL;  ///< controller heartbeat to GCS client
extern const char* const NAV_RUNSTATE_CMD;        ///< GCS mission state command to mission planner
extern const char* const NAV_RUNSTATE_STAT;       ///< Mission planner heartbeat
extern const char* const CAMERA_DISC_CMD;         ///< GCS command to calibrate cameras
extern const char* const CAMERA_DISC_STAT;        ///< vision response to camera calibration cmd
extern const char* const VISION_STAT;             ///< vision heartbeat to GCS
extern const char* const PLANNER_CMD;             ///< GCS commands to mission planner
extern const char* const PLANNER_STAT;            ///< Planner confirmation to GCS
extern const char* const OBST_HEARTBEAT_CHANNEL;  ///< Obstacle det. heartbeat to GCS
extern const char* const TRACKED_STATE;           ///< Vehicle position history sent to vision
extern const char* const IDLE_CHANNEL;            ///< channel on which idle commands are broadcast
extern const char* const START_VISION;            ///< channel on which to send start vision message
extern const char* const PLANE_FIT_CHANNEL;       ///< plane fitting update
extern const char* const VISUAL_ODOMETRY_CHANNEL;  ///< visual odometry updates
extern const char* const
    SIM_TRUE_VISUAL_ODOMETRY_CHANNEL;   ///< "true" visual odometry from simulator
extern const char* const RGBD_CHANNEL;  ///< rgbd images sent to slam
extern const char* const GLOBAL_UPDATE_CHANNEL; ///< channel for global update to kalman filter
}

#endif
