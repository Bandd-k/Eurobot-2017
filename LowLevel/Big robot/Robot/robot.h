#ifndef _ROBOTS_INCLUDED_
#define _ROBOTS_INCLUDED_

#include "Communication.h"
#include "gpio.h"
#include <stdbool.h>

#define WHEEL1_PWM_PIN BTN1_PWM_PIN    //              //TIM3 ch 3  AF2//
#define WHEEL2_PWM_PIN BTN2_PWM_PIN                    //TIM3 ch 4  AF2//
#define WHEEL3_PWM_PIN BTN3_PWM_PIN                    //TIM3 ch 1  AF2//
#define WHEEL4_PWM_PIN BTN4_PWM_PIN                    //TIM3 ch 2  AF2//

#define WHEEL1_DIR_PIN BTN1_DIR_PIN    //              //TIM3 ch 3  AF2//
#define WHEEL2_DIR_PIN BTN2_DIR_PIN                    //TIM3 ch 4  AF2//
#define WHEEL3_DIR_PIN BTN3_DIR_PIN                    //TIM3 ch 1  AF2//
#define WHEEL4_DIR_PIN BTN4_DIR_PIN                    //TIM3 ch 2  AF2//

#define WHEEL1_CH   0
#define WHEEL2_CH   1
#define WHEEL3_CH   2
#define WHEEL4_CH   3

#define ENC1A_PIN ENCODER1A_PIN         //������� 1 � TIM1//AF1
#define ENC1B_PIN ENCODER1A_PIN         //������� 1 B TIM1//AF1
#define ENC2A_PIN ENCODER2A_PIN         //������� 2 � TIM8//AF3
#define ENC2B_PIN ENCODER2A_PIN         //������� 2 B TIM8//AF3
#define ENC3A_PIN ENCODER3A_PIN         //������� 3 � TIM5//AF2
#define ENC3B_PIN ENCODER3A_PIN         //������� 3 B TIM5//AF2
#define ENC4A_PIN ENCODER4A_PIN         //������� 4 � TIM4//AF2
#define ENC4B_PIN ENCODER4A_PIN         //������� 4 B TIM4//AF2

#define SYNC_BYTE 0xFA
#define ADR_BYTE  0xAF
#define HEADER_SIZE  4
#define CHECK_SIZE   2

#define ADC_ANALOG_PIN 0
#define ADC_DIG_INPUT  1
#define ADC_DIG_OUTPUT 2

#define EXTI_BOTH       0
#define EXTI_RISE       1
#define EXTI_FALL       2
#define EXTI_DIG_INPUT  3
#define EXTI_DIG_OUTPUT 4
#define NO_MOTOR 255

#define MAX_DIST_FRONT 520//instead of MAX_DIST
#define MIN_DIST_FRONT 110//instead of MIN_DIST

#define MAX_DIST_BACK 445
#define MIN_DIST_BACK 105

#define MAX_RAW_SENSOR 4095
#define MIN_RAW_SENSOR 35

#define CRITICAL_DIST 200
#define FRONT 7
//#define FRONT_RIGHT 1
//#define BACK_LEFT 2
#define BACK 6

#define threshhold -7

#define IR_LEFT_FRONT   GENERAL_PIN_0
#define IR_LEFT_BACK    GENERAL_PIN_1
#define IR_FRONT        GENERAL_PIN_2
#define IR_BACK         GENERAL_PIN_3
#define IR_RIGHT_FRONT  GENERAL_PIN_4
#define IR_RIGHT_BACK   GENERAL_PIN_5

#define IR_FRONT2       GENERAL_PIN_6
#define IR_BACK2        GENERAL_PIN_7
#pragma pack(push,1)
typedef struct {
  char sync;
  char adress;
  char packLen;
  char command ;
  char * param;
} InPackStruct;

typedef struct {
  char sync;
  char adress;
  char command;
  float robotCoord[4];
  uint16_t checkSum;
} OutPackStruct;
#pragma pack(pop)

//��������� ������ ������������ �������
#pragma pack(push,1)
typedef struct {
  char sync;
  char adress;
  float robotSpeed[3];
  float robotCoord[3];
  uint16_t checkSum;
} encInPackStruct;

typedef struct {
  char sync;
  char adress;
  char Command;
  float robotCoord[4];
  uint16_t checkSum;

} encOutPackStruct;
#pragma pack(pop)



typedef struct {
  char pidEnabled;
  char trackEn;
  char kinemEn;
  char filtering;
  char collisionAvEn;
} robStateStruct;

extern float robotCoordTarget[3];
extern float robotSpeedTarget[3] ;
extern float motorSpeed[4];
extern float motorCoord[4] ;
extern float robotCoord[3] ;
extern float robotSpeed[3] ;
extern robStateStruct curState;
extern encOutPackStruct outEnc;
extern float vTargetGlob[3];

extern uint8_t distance_digital2[8];

extern uint32_t  PWM_DIR[10];
extern uint32_t * PWM_CCR[10];
extern uint32_t * encCnt[4];
extern char  WHEELS[4];
extern uint16_t adcData[10];

extern bool flag;
extern char packLen[0x29];
extern InPackStruct inCommand;
extern char inData[64];
extern char dataIndex;
extern float distanceData[3][6];
void takeadc(float distanceData[][6],int adc_number1,int adc_number2,int adc_number3);
void stopmove();
void checkCollisionAvoid_small(float *rV , float* vTargetGlob);

char setVoltage(char ch, float duty);
char execCommand(InPackStruct* cmd);
void pushByte(char inByte);
char sendAnswer(char cmd,char * param,int paramSize);

extern char startFlag;
#endif
