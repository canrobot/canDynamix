/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "motor.h"

#include "src/PID_v1.h"  
#include "src/FlexiTimer2.h"





typedef struct
{
  int32_t  counter;

  int32_t  speed;
  int32_t  start_counter;  
  
  double   pwm_out;

  uint8_t  enc_pin[2];
  uint8_t  mot_pin[2];

  int8_t   mot_dir;

  // PID
  double enc_speed;
  double goal_speed;
  double pwm_output;
  PID   *p_pid;  
} motor_cfg_t;


motor_cfg_t motor_cfg[2];


void (*motor_isr_callback)(void) = NULL;



static void motorEncoderLeftISR(void);
static void motorEncoderRightISR(void);
static void motorEncoderUpdate(uint8_t ch);
static void motorUpdateISR(void);





void motorBegin(void)
{
  motor_cfg[L_MOTOR].mot_dir    =-1;  // 1 or -1
  motor_cfg[L_MOTOR].enc_pin[0] = 3;  // Interrupt Pin
  motor_cfg[L_MOTOR].enc_pin[1] = 7;
  motor_cfg[L_MOTOR].mot_pin[0] = 9;
  motor_cfg[L_MOTOR].mot_pin[1] = 10;
  
  
  motor_cfg[R_MOTOR].mot_dir    = 1;  // 1 or -1
  motor_cfg[R_MOTOR].enc_pin[0] = 2;  // Interrupt Pin
  motor_cfg[R_MOTOR].enc_pin[1] = 4;
  motor_cfg[R_MOTOR].mot_pin[0] = 6;
  motor_cfg[R_MOTOR].mot_pin[1] = 5;  

  
  for (int i=0; i<2; i++)
  {
    motor_cfg[i].counter = 0;
    motor_cfg[i].speed   = 0;
    motor_cfg[i].pwm_out = 0;
    motor_cfg[i].p_pid   = new PID(&motor_cfg[i].enc_speed, &motor_cfg[i].pwm_output, &motor_cfg[i].goal_speed, 30, 20, 0, DIRECT);
    motor_cfg[i].start_counter = 0;

    pinMode(motor_cfg[i].enc_pin[0], INPUT_PULLUP);
    pinMode(motor_cfg[i].enc_pin[1], INPUT_PULLUP);      

    analogWrite(motor_cfg[i].mot_pin[0], 0);
    analogWrite(motor_cfg[i].mot_pin[1], 0);   
    
    motor_cfg[i].p_pid->SetSampleTime(10);
    motor_cfg[i].p_pid->SetOutputLimits(-255, 255);
    motor_cfg[i].p_pid->SetMode(AUTOMATIC);    
  }  

  attachInterrupt(digitalPinToInterrupt(motor_cfg[L_MOTOR].enc_pin[0]), motorEncoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_cfg[R_MOTOR].enc_pin[0]), motorEncoderRightISR, CHANGE);


  FlexiTimer2::set(10, motorUpdateISR); 
  FlexiTimer2::start();  
}

void motorSetCallback(void (*func)(void))
{
  motor_isr_callback = func;
}

int32_t motorGetSpeed(uint8_t ch)
{
  return motor_cfg[ch].speed;
}

int32_t motorGetCounter(uint8_t ch)
{
  return motor_cfg[ch].counter;
}

int32_t motorGetGoalSpeed(uint8_t ch)
{
  return (int32_t)motor_cfg[ch].goal_speed;
}

void motorSetSpeed(uint8_t ch, int16_t speed)
{
  motor_cfg[ch].goal_speed = (double)speed;
}

void motorMoveSpeed(int16_t left_speed, int16_t right_speed)
{
  motor_cfg[L_MOTOR].goal_speed = (double)left_speed;
  motor_cfg[R_MOTOR].goal_speed = (double)right_speed;
}


void motorSetPwm(uint8_t ch, int16_t pwm_data )
{
  uint16_t pwm_out;

  if (pwm_data >= 0)
  {
    pwm_out = pwm_data;
    analogWrite(motor_cfg[ch].mot_pin[0], pwm_out);
    analogWrite(motor_cfg[ch].mot_pin[1], 0);     
  }
  else
  {
    pwm_out = -pwm_data;
    analogWrite(motor_cfg[ch].mot_pin[0], 0);
    analogWrite(motor_cfg[ch].mot_pin[1], pwm_out);         
  }  
}

// Motor Update
//
void motorUpdateISR(void)
{
  sei();
  for (int i=0; i<2; i++)
  {
    motor_cfg[i].speed = motor_cfg[i].counter - motor_cfg[i].start_counter;
    motor_cfg[i].start_counter = motor_cfg[i].counter;
    motor_cfg[i].enc_speed = (double)motor_cfg[i].speed;

    if (motor_cfg[i].p_pid->ComputeISR())
    {
      if (motor_cfg[i].goal_speed == 0)
      {
        motorSetPwm(i, 0);
      }
      else
      {
        motorSetPwm(i, (int16_t)motor_cfg[i].pwm_output);
      }
    }      
  }

  if (motor_isr_callback != NULL)
  {
    (*motor_isr_callback)();
  }
}

void motorEncoderLeftISR(void)
{
  sei();
  motorEncoderUpdate(L_MOTOR);
}

void motorEncoderRightISR(void)
{
  sei();
  motorEncoderUpdate(R_MOTOR);
}

void motorEncoderUpdate(uint8_t ch)
{
  uint8_t enc_bit = 0;
  
  if (digitalRead(motor_cfg[ch].enc_pin[0]) == HIGH)
  {
    enc_bit |= (1<<0);
  }
  if (digitalRead(motor_cfg[ch].enc_pin[1]) == HIGH)
  {
    enc_bit |= (1<<1);
  }

  switch(enc_bit)
  {
    case 0x01:
    case 0x02:
      motor_cfg[ch].counter -= motor_cfg[ch].mot_dir;
      break;

    default:
      motor_cfg[ch].counter += motor_cfg[ch].mot_dir;
      break;
  }  
}