/*
 * tasks.h - Pontos de entrada dos cinco blocos de firmware (Tabela 6).
 */
#ifndef TASKS_H
#define TASKS_H

void bt_command_start(void);   /* por evento  - SPP, parser, timeout */
void sonar_start(void);        /* 100 ms      - quatro HC-SR04       */
void odometry_start(void);     /*  20 ms      - encoders e IMU       */
void control_start(void);      /*  50 ms      - seguranca e ponte H  */
void telemetry_start(void);    /* 200 ms      - Wi-Fi e MQTT         */

/* Acorda a tarefa de controle fora do periodo: parada prioritaria (RF03) e
 * timeout de comando (RF08) nao podem esperar o proximo ciclo de 50 ms. */
void control_notify(void);

#endif /* TASKS_H */
