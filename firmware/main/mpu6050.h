#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t mpu6050_init(void);

/* Averages samples at rest to remove the gyro bias. The rover must be still. */
void mpu6050_calibrate_gyro(void);

/* Rover frame (x forward, y left, z up), gyro bias removed. */
esp_err_t mpu6050_read(float accel_m_s2[3], float gyro_rad_s[3]);

#endif /* MPU6050_H */
