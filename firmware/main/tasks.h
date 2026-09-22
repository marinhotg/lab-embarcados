#ifndef TASKS_H
#define TASKS_H

void bt_command_start(void);
void sonar_start(void);
void odometry_start(void);
void control_start(void);
void telemetry_start(void);

/* Wakes the control task off-period: a stop command or a command timeout must
 * not wait for the next 50 ms cycle. */
void control_notify(void);

#endif /* TASKS_H */
