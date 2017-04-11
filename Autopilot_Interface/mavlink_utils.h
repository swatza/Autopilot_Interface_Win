#pragma once

#include "common/mavlink.h"

/**
* Defines for mavlink_set_position_target_local_ned_t.type_mask
*
* Bitmask to indicate which dimensions should be ignored by the vehicle
*
* a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
* the setpoint dimensions should be ignored.
*
* If bit 10 is set the floats afx afy afz should be interpreted as force
* instead of acceleration.
*
* Mapping:
* bit 1: x,
* bit 2: y,
* bit 3: z,
* bit 4: vx,
* bit 5: vy,
* bit 6: vz,
* bit 7: ax,
* bit 8: ay,
* bit 9: az,
* bit 10: is force setpoint,
* bit 11: yaw,
* bit 12: yaw rate
* remaining bits unused
*
* Combine bitmasks with bitwise &
*
* Example for position and yaw angle:
* uint16_t type_mask =
*     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
*     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
*/

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);