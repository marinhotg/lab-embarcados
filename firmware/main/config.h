/* Pinos e constantes de calibracao. Unico ponto de ajuste do firmware. */
#ifndef CONFIG_H
#define CONFIG_H

#define CARRINHO_ID            "01"
#define BT_DEVICE_NAME         "Carrinho"
#define TELEMETRY_TOPIC        "carrinho/" CARRINHO_ID "/telemetria"
#define SCHEMA_VERSAO          1

/* Pinos mapeados para a WEMOS LOLIN32 v1.0.0, que expoe 24 GPIOs e NAO tem
 * 16 nem 17. Dos cinco pinos de strapping (0 2 5 12 15) so o 5 e usado, e como
 * saida - seguro, porque no reset ele fica com pull-up interno. Livres: 0, 2,
 * 12, 15. */

#define PIN_PWM_LEFT           25   /* ENA */
#define PIN_DIR_LEFT_A         27   /* IN1 */
#define PIN_DIR_LEFT_B         26   /* IN2 */
#define PIN_PWM_RIGHT          13   /* ENB */
#define PIN_DIR_RIGHT_A        32   /* IN3 */
#define PIN_DIR_RIGHT_B        33   /* IN4 */

#define SONAR_COUNT            4
#define SONAR_FRENTE           0
#define SONAR_TRAS             1
#define SONAR_ESQUERDA         2
#define SONAR_DIREITA          3

#define PIN_SONAR_TRIGGERS     { 14,  4,  5, 23 }
#define PIN_SONAR_ECHOS        { 18, 19, 36, 39 }

/* Geometria de cada sensor no referencial do carrinho: x para a frente,
 * y para a esquerda, angulo positivo anti-horario. Espelhado em map_client.py. */
#define SONAR_OFFSET_X_M       { 0.10f,  -0.10f,  0.00f,  0.00f }
#define SONAR_OFFSET_Y_M       { 0.00f,   0.00f,  0.07f, -0.07f }
#define SONAR_BEARING_RAD      { 0.0f, 3.14159265f, 1.57079633f, -1.57079633f }

/* 34 e 35 sao input-only e sem pull-up interno: polarizacao externa e obrigatoria. */
#define PIN_ENCODER_LEFT       34
#define PIN_ENCODER_RIGHT      35

#define PIN_I2C_SDA            21
#define PIN_I2C_SCL            22
#define I2C_FREQ_HZ            400000
#define MPU6050_ADDR           0x68

/* Calibrar na Aula 12. Os valores abaixo sao estimativas, nao medidas. */
#define WHEEL_RADIUS_M         0.0325f   /* r */
#define WHEEL_BASE_M           0.150f    /* b, bitola */
#define ENCODER_PULSES_PER_REV 20.0f     /* N, ranhuras do disco */
#define V_MAX_MPS              0.20f     /* escala do byte 0x01 */
#define OMEGA_MAX_RAD_S        1.50f     /* escala do byte 0x02, nao especificada no documento */
#define MOTOR_DUTY_MIN         0.30f     /* zona morta: abaixo disso o motor nao gira */
#define MOTOR_DUTY_MAX         1.00f

/* Constante de tempo do filtro = dt*alpha/(1-alpha). Precisa cobrir a curva
 * inteira, senao o escorregamento nao e rejeitado: com 0,98 o erro fica em
 * 14,7 graus, no limite dos 15 da Tabela 1. Ver firmware/test. */
#define COMPLEMENTARY_ALPHA    0.995f

#define PWM_FREQ_HZ            1000
#define PWM_RESOLUTION_BITS    10
#define PWM_DUTY_MAX_RAW       ((1 << PWM_RESOLUTION_BITS) - 1)

#define PERIOD_SONAR_MS        100   /* um sensor por ativacao -> varredura de 400 ms */
#define PERIOD_ODOMETRY_MS     20
#define PERIOD_CONTROL_MS      50
#define PERIOD_TELEMETRY_MS    200

#define COMMAND_TIMEOUT_MS     450   /* RNF03 exige parada em ate 500 ms */
#define OBSTACLE_BRAKE_M       0.30f
#define SONAR_MAX_AGE_MS       500

#define SONAR_ECHO_TIMEOUT_MS  30    /* ~5 m */
#define SONAR_MIN_VALID_M      0.02f
#define SONAR_MAX_VALID_M      4.00f
#define SOUND_SPEED_M_S        343.0f
#define SONAR_MEDIAN_WINDOW    3

/* Controle acima de MQTT (secao 4.5). */
#define PRIO_CONTROL           6
#define PRIO_ODOMETRY          5
#define PRIO_BT_COMMAND        5
#define PRIO_SONAR             4
#define PRIO_TELEMETRY         3

#endif /* CONFIG_H */
