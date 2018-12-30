#ifndef _CONTROLSTATE_
#define _CONTROLSTATE_

namespace maav
{
namespace gnc
{
enum class ControlState
{
    STANDBY = 0,
    XBOX_CONTROL,
    TAKEOFF,
    LAND,
    EMS_LAND,
    FLIGHT,
    ARMING,
    DISARMING,
    KILLSWITCH
};

}  // maav
}  // gnc

#endif  // _CONTROLSTATE_