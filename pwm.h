#ifndef PWM_H
#define PWM_H

void pwm_init(void);
int comp_duty(int percentage);
void pwm_start(void);
void pwm_set_velocities(int forward_speed, int yaw_rate);
void pwm_stop(void);

#endif
