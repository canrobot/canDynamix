#ifndef _MBLOCK_H_
#define _MBLOCK_H_





#define VERSION               0
#define ULTRASONIC_SENSOR     1
#define TEMPERATURE_SENSOR    2
#define LIGHT_SENSOR          3
#define POTENTIONMETER        4
#define JOYSTICK              5
#define GYRO                  6
#define SOUND_SENSOR          7
#define RGBLED                8
#define SEVSEG                9
#define MOTOR                 100 // for canDynamix
#define SERVO                 11
#define ENCODER               12
#define IR                    13
#define PIRMOTION             15
#define INFRARED              16
#define LINEFOLLOWER          17
#define SHUTTER               20
#define LIMITSWITCH           21
#define BUTTON                22
#define DIGITAL               30
#define ANALOG                31
#define PWM                   32
#define SERVO_PIN             33
#define TONE                  34
#define PULSEIN               35
#define ULTRASONIC_ARDUINO    36
#define STEPPER               40
#define ENCODER               41
#define TIMER                 50
#define L_SERVO               37
#define R_SERVO               38
#define BUZZER                39
#define LED                   42

#define GET   1
#define RUN   2
#define RESET 4
#define START 5

typedef struct
{
  uint8_t state;

  uint8_t length;
  uint8_t rx_index;

  uint8_t idx;
  uint8_t action;;
  uint8_t device;;
  uint8_t port;
  uint8_t slot;

  char    buffer[52];
} mblcok_packet_t;


void mblockBegin(uint32_t baud);
bool mblockUpdate(void);

void mblockSetReadCallback(void (*func)(mblcok_packet_t *p_packet, uint8_t device));
void mblockSetRunCallback(void (*func)(mblcok_packet_t *p_packet, uint8_t device));

void  mblockSendByte(char c);
void  mblockSendString(String s);
void  mblockSendFloat(float value);
void  mblockSendShort(double value);
void  mblockSendDouble(double value);
short mblockReadShort(int idx);
float mblockReadFloat(int idx);
uint8_t mblockReadBuffer(int index);

#endif

