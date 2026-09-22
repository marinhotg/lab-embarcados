/* Pin map and calibration constants: the single place to tune the firmware. */
#ifndef CONFIG_H
#define CONFIG_H

#define ROVER_ID               "01"
#define BT_DEVICE_NAME         "Rover"
#define TELEMETRY_TOPIC        "rover/" ROVER_ID "/telemetry"
#define SCHEMA_VERSION         1

/* WEMOS LOLIN32 v1.0.0 has no GPIO 16/17. Of the strapping pins (0 2 5 12 15)
 * only 5 is used, as an output, which is safe because it boots pulled up. */

#define PIN_PWM_LEFT           25   /* ENA */
#define PIN_DIR_LEFT_A         27   /* IN1 */
#define PIN_DIR_LEFT_B         26   /* IN2 */
#define PIN_PWM_RIGHT          13   /* ENB */
#define PIN_DIR_RIGHT_A        32   /* IN3 */
#define PIN_DIR_RIGHT_B        33   /* IN4 */

#define SONAR_COUNT            4
#define SONAR_FRONT            0
#define SONAR_REAR             1
#define SONAR_LEFT             2
#define SONAR_RIGHT            3

#define PIN_SONAR_TRIGGERS     { 14,  4,  5, 23 }
#define PIN_SONAR_ECHOS        { 18, 19, 36, 39 }

/* Rover frame: x forward, y left, angles counter-clockwise.
 * Mirrored in client/map_client.py. */
#define SONAR_OFFSET_X_M       { 0.10f,  -0.10f,  0.00f,  0.00f }
#define SONAR_OFFSET_Y_M       { 0.00f,   0.00f,  0.07f, -0.07f }
#define SONAR_BEARING_RAD      { 0.0f, 3.14159265f, 1.57079633f, -1.57079633f }

/* Input-only pins without internal pull-up: external pull-up is mandatory. */
#define PIN_ENCODER_LEFT       34
#define PIN_ENCODER_RIGHT      35

#define PIN_I2C_SDA            21
#define PIN_I2C_SCL            22
#define I2C_FREQ_HZ            400000
#define MPU6050_ADDR           0x68

/* Initial estimates, not yet measured. */
#define WHEEL_RADIUS_M         0.0325f
#define WHEEL_BASE_M           0.150f
#define ENCODER_PULSES_PER_REV 20.0f
#define V_MAX_MPS              0.20f     /* full scale of command 0x01 */
#define OMEGA_MAX_RAD_S        1.50f     /* full scale of command 0x02 */
#define MOTOR_DUTY_MIN         0.30f     /* below this the motor stalls */
#define MOTOR_DUTY_MAX         1.00f

/* Filter time constant is dt*alpha/(1-alpha) and must span a whole turn to
 * reject wheel slip: 0.98 leaves a 14.7 deg error. See firmware/test. */
#define COMPLEMENTARY_ALPHA    0.995f

#define PWM_FREQ_HZ            1000
#define PWM_RESOLUTION_BITS    10
#define PWM_DUTY_MAX_RAW       ((1 << PWM_RESOLUTION_BITS) - 1)

#define PERIOD_SONAR_MS        100   /* one sensor per tick, full sweep in 400 ms */
#define PERIOD_ODOMETRY_MS     20
#define PERIOD_CONTROL_MS      50
#define PERIOD_TELEMETRY_MS    200

/* Bench only: with 0, moving forward no longer requires a front sonar reading.
 * A missing reading counts as invalid and would otherwise block all forward
 * motion. */
#define SAFETY_REQUIRE_FRONT_SONAR 1

#define WIFI_RETRY_MS          5000

#define COMMAND_TIMEOUT_MS     450   /* the spec requires a stop within 500 ms */
#define OBSTACLE_BRAKE_M       0.30f
#define SONAR_MAX_AGE_MS       500

#define SONAR_ECHO_TIMEOUT_MS  30    /* ~5 m */
#define SONAR_MIN_VALID_M      0.02f
#define SONAR_MAX_VALID_M      4.00f
#define SOUND_SPEED_M_S        343.0f
#define SONAR_MEDIAN_WINDOW    3

#define PRIO_CONTROL           6
#define PRIO_ODOMETRY          5
#define PRIO_BT_COMMAND        5
#define PRIO_SONAR             4
#define PRIO_TELEMETRY         3

#endif /* CONFIG_H */
