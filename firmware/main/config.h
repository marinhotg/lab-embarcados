/*
 * config.h - Mapeamento de pinos e constantes de calibracao.
 *
 * Unico lugar do firmware onde se mexe para ajustar hardware ou calibracao
 * (RNF08: configuracao centralizada). Pinos conforme a Tabela 5 do documento
 * de especificacao.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ========================================================================
 * Identificacao
 * ===================================================================== */
#define CARRINHO_ID            "01"
#define BT_DEVICE_NAME         "Carrinho"
#define TELEMETRY_TOPIC        "carrinho/" CARRINHO_ID "/telemetria"
#define SCHEMA_VERSAO          1

/* ========================================================================
 * Pinos - ponte H (Tabela 5)
 *
 * Canal A = par esquerdo, canal B = par direito. ENA/ENB recebem PWM; os
 * pares de pinos de sentido definem a polaridade de cada canal.
 * ===================================================================== */
#define PIN_PWM_LEFT           25   /* ENA */
#define PIN_DIR_LEFT_A         26
#define PIN_DIR_LEFT_B         27
#define PIN_PWM_RIGHT          33   /* ENB */
#define PIN_DIR_RIGHT_A        32
#define PIN_DIR_RIGHT_B        13

/* ========================================================================
 * Pinos - HC-SR04 (Tabela 5). Ordem: frente, tras, esquerda, direita.
 *
 * ATENCAO DE BANCADA: GPIO 15 (trigger) e GPIO 5 (echo) sao strapping pins.
 * GPIO 15 em nivel baixo no reset apenas silencia o log de boot. GPIO 5 preso
 * em baixo pelo modulo no reset e risco real - se a placa nao inicializar,
 * trocar por GPIO 12 ou 4 aqui resolve sem tocar em mais nada.
 * ===================================================================== */
#define SONAR_COUNT            4
#define SONAR_FRENTE           0
#define SONAR_TRAS             1
#define SONAR_ESQUERDA         2
#define SONAR_DIREITA          3

#define PIN_SONAR_TRIGGERS     { 14, 15, 16, 17 }
#define PIN_SONAR_ECHOS        { 18, 19, 23,  5 }

/* Posicao de cada sensor no referencial do carrinho, usada pelo cliente de
 * mapa para projetar o ponto detectado. x aponta para a frente, y para a
 * esquerda, angulo positivo anti-horario.  CALIBRAR na Aula 12. */
#define SONAR_OFFSET_X_M       { 0.10f,  -0.10f,  0.00f,  0.00f }
#define SONAR_OFFSET_Y_M       { 0.00f,   0.00f,  0.07f, -0.07f }
#define SONAR_BEARING_RAD      { 0.0f, 3.14159265f, 1.57079633f, -1.57079633f }

/* ========================================================================
 * Pinos - encoders e I2C (Tabela 5)
 *
 * GPIO 34/35 sao input-only e NAO tem pull-up interno. O pull-up externo em
 * 3,3 V e obrigatorio, nao opcional (RNF06).
 * ===================================================================== */
#define PIN_ENCODER_LEFT       34
#define PIN_ENCODER_RIGHT      35

#define PIN_I2C_SDA            21
#define PIN_I2C_SCL            22
#define I2C_FREQ_HZ            400000
#define MPU6050_ADDR           0x68

/* ========================================================================
 * Geometria e calibracao
 *
 * >>> TODOS OS VALORES ABAIXO SAO PLACEHOLDERS. <<<
 * Medir e ajustar na Aula 12 (calibracao de raio efetivo, bitola e pulsos
 * por volta), conforme o cronograma.
 * ===================================================================== */
#define WHEEL_RADIUS_M         0.0325f   /* r  - raio efetivo da roda    */
#define WHEEL_BASE_M           0.150f    /* b  - bitola, distancia entre lados */
#define ENCODER_PULSES_PER_REV 20.0f     /* N  - ranhuras do disco       */

/* v_max e omega_max sao as escalas do protocolo Bluetooth: o byte de valor
 * (-100 a 100) e um percentual destes. omega_max nao e especificado no
 * documento; 1,5 rad/s e uma estimativa a confirmar em bancada. */
#define V_MAX_MPS              0.20f
#define OMEGA_MAX_RAD_S        1.50f

/* Zona morta do motor: abaixo deste duty o motor DC nao vence o atrito.
 * A conversao velocidade->duty mapeia o intervalo util [MIN, MAX]. */
#define MOTOR_DUTY_MIN         0.30f
#define MOTOR_DUTY_MAX         1.00f

/* Filtro complementar da equacao (6). alpha alto confia mais no giroscopio.
 *
 * A constante de tempo e dt*alpha/(1-alpha). Para uma curva de 90 graus em
 * 1,5 s com 30 graus de escorregamento, o erro do yaw fundido contra a
 * verdade fica assim (medido em test_odom, ver README):
 *
 *     alpha    tau      erro
 *     0,90    0,18 s   +26,4 graus
 *     0,98    0,98 s   +14,7 graus   <- no limite dos 15 graus da Tabela 1
 *     0,995   3,98 s    +5,1 graus
 *     0,999  19,98 s    +1,1 graus
 *
 * Um filtro complementar so rejeita escorregamento transitorio: em regime
 * permanente ele acompanha a taxa da odometria. Por isso tau precisa cobrir a
 * duracao da curva inteira. O preco de subir alpha e deixar o desvio do
 * giroscopio acumular entre manobras - por isso 0,995 e nao 0,999. */
#define COMPLEMENTARY_ALPHA    0.995f

/* ========================================================================
 * PWM (LEDC)
 *
 * Modelo da ponte H ainda nao identificado (secao 2.2 do documento). Assume-se
 * comportamento tipo L298N: ENA/ENB aceitam PWM. 1 kHz e conservador e audivel;
 * se a ponte permitir, 5-20 kHz elimina o ruido.
 * ===================================================================== */
#define PWM_FREQ_HZ            1000
#define PWM_RESOLUTION_BITS    10
#define PWM_DUTY_MAX_RAW       ((1 << PWM_RESOLUTION_BITS) - 1)

/* ========================================================================
 * Periodos das tarefas (Tabela 6) e limites temporais (Tabela 3)
 * ===================================================================== */
#define PERIOD_SONAR_MS        100   /* um sensor por ativacao -> varredura 400 ms */
#define PERIOD_ODOMETRY_MS     20
#define PERIOD_CONTROL_MS      50
#define PERIOD_TELEMETRY_MS    200

/* RNF03: parada aplicada ate 500 ms apos o ultimo pacote valido. O disparo em
 * 450 ms deixa margem para o ciclo de controle de 50 ms. */
#define COMMAND_TIMEOUT_MS     450

/* RF07 / secao 3.2 */
#define OBSTACLE_BRAKE_M       0.30f  /* frear abaixo desta distancia frontal */
#define SONAR_MAX_AGE_MS       500    /* leitura frontal mais velha impede avanco */

/* ========================================================================
 * HC-SR04
 * ===================================================================== */
#define SONAR_ECHO_TIMEOUT_MS  30      /* ~5 m; ausencia de eco               */
#define SONAR_MIN_VALID_M      0.02f
#define SONAR_MAX_VALID_M      4.00f
#define SOUND_SPEED_M_S        343.0f
#define SONAR_MEDIAN_WINDOW    3

/* ========================================================================
 * Prioridades das tarefas
 * Controle tem prioridade sobre MQTT (secao 4.5).
 * ===================================================================== */
#define PRIO_CONTROL           6
#define PRIO_ODOMETRY          5
#define PRIO_BT_COMMAND        5
#define PRIO_SONAR             4
#define PRIO_TELEMETRY         3

#endif /* CONFIG_H */
