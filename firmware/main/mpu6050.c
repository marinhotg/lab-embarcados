#include "mpu6050.h"

#include <math.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"

static const char *TAG = "mpu6050";

#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

/* Fundos de escala escolhidos: +-2 g e +-250 graus/s.
 * O carrinho anda a 0,20 m/s; faixas maiores so perderiam resolucao. */
#define ACCEL_LSB_PER_G      16384.0f
#define GYRO_LSB_PER_DEG_S   131.0f
#define GRAVITY_M_S2         9.80665f

#define CALIB_SAMPLES        200

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static float s_gyro_bias[3];

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, 100);
}

esp_err_t mpu6050_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = PIN_I2C_SDA,
        .scl_io_num                   = PIN_I2C_SCL,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev), TAG, "dev");

    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(read_regs(REG_WHO_AM_I, &who, 1), TAG, "who_am_i");
    if (who != 0x68 && who != 0x70) {
        ESP_LOGE(TAG, "WHO_AM_I inesperado: 0x%02x", who);
        return ESP_ERR_NOT_FOUND;
    }

    /* Clock no eixo X do giroscopio: mais estavel que o oscilador interno. */
    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, 0x01), TAG, "wake");
    vTaskDelay(pdMS_TO_TICKS(50));

    /* DLPF em 44 Hz: corta a vibracao dos motores sem atrasar a leitura de 20 ms. */
    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, 0x03), TAG, "dlpf");
    ESP_RETURN_ON_ERROR(write_reg(REG_SMPLRT_DIV, 0x04), TAG, "rate");   /* 200 Hz */
    ESP_RETURN_ON_ERROR(write_reg(REG_GYRO_CONFIG, 0x00), TAG, "gyro_fs");
    ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG, 0x00), TAG, "accel_fs");

    ESP_LOGI(TAG, "IMU inicializada");
    return ESP_OK;
}

/* Le os 14 bytes de medida de uma vez: ax ay az temp gx gy gz. */
static esp_err_t read_raw(int16_t accel[3], int16_t gyro[3])
{
    uint8_t buf[14];
    esp_err_t err = read_regs(REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }

    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((buf[i * 2] << 8) | buf[i * 2 + 1]);
        gyro[i]  = (int16_t)((buf[8 + i * 2] << 8) | buf[9 + i * 2]);
    }
    return ESP_OK;
}

void mpu6050_calibrate_gyro(void)
{
    float sum[3] = { 0.0f, 0.0f, 0.0f };
    int   taken  = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t accel[3], gyro[3];
        if (read_raw(accel, gyro) == ESP_OK) {
            for (int a = 0; a < 3; a++) {
                sum[a] += gyro[a] / GYRO_LSB_PER_DEG_S * (float)M_PI / 180.0f;
            }
            taken++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (taken == 0) {
        ESP_LOGE(TAG, "calibracao do giroscopio falhou");
        return;
    }

    for (int a = 0; a < 3; a++) {
        s_gyro_bias[a] = sum[a] / taken;
    }

    ESP_LOGI(TAG, "desvio do giroscopio: %.4f %.4f %.4f rad/s",
             s_gyro_bias[0], s_gyro_bias[1], s_gyro_bias[2]);
}

esp_err_t mpu6050_read(float accel_m_s2[3], float gyro_rad_s[3])
{
    int16_t raw_accel[3], raw_gyro[3];
    esp_err_t err = read_raw(raw_accel, raw_gyro);
    if (err != ESP_OK) {
        return err;
    }

    /* Identidade assume a IMU deitada e alinhada com o chassi. Se a fixacao
     * ficar girada, corrigir aqui e so aqui (calibracao geometrica, secao 2.3). */
    for (int a = 0; a < 3; a++) {
        accel_m_s2[a] = raw_accel[a] / ACCEL_LSB_PER_G * GRAVITY_M_S2;
        gyro_rad_s[a] = raw_gyro[a] / GYRO_LSB_PER_DEG_S * (float)M_PI / 180.0f
                        - s_gyro_bias[a];
    }

    return ESP_OK;
}
