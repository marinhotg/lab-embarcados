/* Driver minimo do MPU6050: so aceleracao e velocidade angular. */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>

#include "esp_err.h"

/* Inicializa o barramento I2C, acorda o sensor e confere o WHO_AM_I. */
esp_err_t mpu6050_init(void);

/* Media de amostras em repouso para remover o desvio do giroscopio.
 * Chamada uma vez no boot - o carrinho precisa estar parado. */
void mpu6050_calibrate_gyro(void);

/* accel em m/s2, gyro em rad/s, ja nos eixos do carrinho
 * (x frente, y esquerda, z cima) e com o desvio do giroscopio removido. */
esp_err_t mpu6050_read(float accel_m_s2[3], float gyro_rad_s[3]);

#endif /* MPU6050_H */
