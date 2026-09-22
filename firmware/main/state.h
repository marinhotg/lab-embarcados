/*
 * state.h - Estado compartilhado entre as tarefas.
 *
 * Regra de escrita (um dono por campo, verificavel por inspecao):
 *
 *   bt_command  escreve  cmd
 *   sonar       escreve  sonar[]
 *   odometry    escreve  pose, wheel, imu
 *   control     escreve  drive, safety   <- unico escritor da ponte H
 *   telemetry   nao escreve nada
 *
 * Leitura sempre por state_snapshot(), que copia tudo sob mutex e devolve.
 * Nenhuma tarefa pode segurar o mutex durante operacao de rede ou espera de
 * eco - e isso que garante o RNF07.
 */
#ifndef STATE_H
#define STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

typedef struct {
    float   v_ref_mps;        /* velocidade linear comandada     */
    float   w_ref_rad_s;      /* velocidade de rotacao comandada */
    int64_t last_packet_us;   /* recepcao do ultimo pacote valido */
    bool    link_connected;   /* SPP conectado                    */
} cmd_state_t;

typedef struct {
    float   dist_m;           /* mediana das ultimas leituras validas */
    float   raw_m;            /* ultima leitura bruta valida          */
    int64_t stamp_us;         /* instante da ultima leitura valida    */
    bool    valid;            /* ja houve pelo menos uma leitura      */
} sonar_state_t;

typedef struct {
    float x_m;
    float y_m;
    float yaw_rad;            /* yaw fundido (odometria + giroscopio) */
    float yaw_odom_rad;       /* yaw so por encoder, mantido separado */
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
    int   dir_left;           /* -1 re, 0 parado, +1 avanco */
    int   dir_right;
    float duty_left;          /* 0.0 a 1.0 */
    float duty_right;
} drive_state_t;

typedef struct {
    bool timeout_active;      /* sem pacote valido ha mais de COMMAND_TIMEOUT_MS */
    bool auto_brake;          /* obstaculo frontal abaixo de OBSTACLE_BRAKE_M   */
    bool front_invalid;       /* leitura frontal ausente ou velha demais        */
} safety_state_t;

typedef struct {
    cmd_state_t    cmd;
    sonar_state_t  sonar[SONAR_COUNT];
    pose_state_t   pose;
    wheel_state_t  wheel;
    imu_state_t    imu;
    drive_state_t  drive;
    safety_state_t safety;
} carrinho_state_t;

void state_init(void);

/* Copia consistente de todo o estado. Usado pela telemetria e pelo controle. */
void state_snapshot(carrinho_state_t *out);

/* Acesso direto para os donos de cada campo. Sempre em par, e sempre curto. */
carrinho_state_t *state_lock(void);
void state_unlock(void);

/* Tempo desde a inicializacao, em milissegundos (campo timestamp_ms do JSON). */
int64_t state_uptime_ms(void);

#endif /* STATE_H */
