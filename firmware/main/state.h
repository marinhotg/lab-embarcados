/* Shared state. Each field has a single writer: bt_command owns cmd, sonar owns
 * sonar[], odometry owns pose/wheel/imu, control owns drive/safety and is the
 * only task touching the H-bridge. Nobody holds the mutex across network I/O
 * or echo waits, so control is never blocked by telemetry. */
#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

typedef struct {
    float   v_ref_mps;
    float   w_ref_rad_s;
    int64_t last_packet_us;
    bool    link_connected;
} cmd_state_t;

typedef struct {
    float   dist_m;           /* median of the last valid readings */
    float   raw_m;            /* last valid reading                */
    int64_t stamp_us;
    bool    valid;            /* at least one reading so far       */
} sonar_state_t;

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;            /* encoder + gyro fusion */
    float yaw_odom_rad;       /* encoders only         */
} pose_state_t;

typedef struct {
    float v_left_mps;
    float v_right_mps;
    float v_linear_mps;
} wheel_state_t;

typedef struct {
    float accel_m_s2[3];
    float gyro_rad_s[3];
    bool  valid;
    int64_t stamp_us;
} imu_state_t;

typedef struct {
    int   dir_left;           /* -1 reverse, 0 stopped, +1 forward */
    int   dir_right;
    float duty_left;          /* 0.0 to 1.0 */
    float duty_right;
} drive_state_t;

typedef struct {
    bool timeout_active;
    bool auto_brake;
    bool front_invalid;       /* front reading missing or stale */
} safety_state_t;

typedef struct {
    cmd_state_t    cmd;
    sonar_state_t  sonar[SONAR_COUNT];
    pose_state_t   pose;
    wheel_state_t  wheel;
    imu_state_t    imu;
    drive_state_t  drive;
    safety_state_t safety;
} rover_state_t;

void state_init(void);

void state_snapshot(rover_state_t *out);

/* For field owners only. Always paired, always short. */
rover_state_t *state_lock(void);
void state_unlock(void);

#endif /* STATE_H */
