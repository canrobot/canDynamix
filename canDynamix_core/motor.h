#include <Arduino.h>

#define CW		0
#define CCW		1
#define Motor_Lpin1	3
#define Motor_Lpin2	5
#define Motor_Rpin1	6
#define Motor_Rpin2	9

void Motor_Left(int direction, int speed);
void Motor_Right(int direction, int speed);
void n20_Motor(int leftMotorSpeed, int rightMotorSpeed);
void Motor_Stop();
