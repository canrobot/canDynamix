#ifndef _MOTOR_H_
#define _MOTOR_H_



#define L_MOTOR    0
#define R_MOTOR    1





void motorBegin(void);
void motorSetCallback(void (*func)(void));

int32_t motorGetSpeed(uint8_t ch);
int32_t motorGetCounter(uint8_t ch);
int32_t motorGetGoalSpeed(uint8_t ch);

void motorSetSpeed(uint8_t ch, int16_t speed);
void motorSetPwm(uint8_t ch, int16_t pwm_data);
void motorMoveSpeed(int16_t left_speed, int16_t right_speed);

#endif

