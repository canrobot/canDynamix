#include "motor.h"

void Motor_Left(int direction, int speed) {
  if(direction == CW) {
    analogWrite(Motor_Lpin1, speed);
    analogWrite(Motor_Lpin2, 0);
  }
  else if(direction == CCW) {
    analogWrite(Motor_Lpin1, 0);
    analogWrite(Motor_Lpin2, speed);
  }
  else {
    analogWrite(Motor_Lpin1, 0);
    analogWrite(Motor_Lpin2, 0);
  }
}

void Motor_Right(int direction, int speed) {
  if(direction == CW) {
    analogWrite(Motor_Rpin1, speed);
    analogWrite(Motor_Rpin2, 0);
  }
  else if(direction == CCW) {
    analogWrite(Motor_Rpin1, 0);
    analogWrite(Motor_Rpin2, speed);
  }
  else {
    analogWrite(Motor_Rpin1, 0);
    analogWrite(Motor_Rpin2, 0);
  }
}

void n20_Motor(int leftMotorSpeed, int rightMotorSpeed){
  if( leftMotorSpeed > 0){
	analogWrite(Motor_Lpin1, leftMotorSpeed);
	analogWrite(Motor_Lpin2, 0);
    }else{
    	analogWrite(Motor_Lpin1, 0);
	analogWrite(Motor_Lpin2, abs(leftMotorSpeed));
    }

    if( rightMotorSpeed > 0){
	analogWrite(Motor_Rpin1, rightMotorSpeed);
	analogWrite(Motor_Rpin2, 0);
    }else{
	analogWrite(Motor_Rpin1, 0);
	analogWrite(Motor_Rpin2, abs(rightMotorSpeed));
    }

	


}


void Motor_Stop() {
  analogWrite(Motor_Lpin1, 0);
  analogWrite(Motor_Lpin2, 0);
  analogWrite(Motor_Rpin1, 0);
  analogWrite(Motor_Rpin2, 0);
}
