/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include <stdbool.h>
#include "gpio.h"
#include "Dynamixel_control.h"

#define MBPS 0x01

void delay(int n){
    while(n>0)n--;
}

void RCC_Config(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}

void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO1;
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO1.GPIO_Mode = GPIO_Mode_AF;
    GPIO1.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO1.GPIO_OType = GPIO_OType_PP;
    GPIO1.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PC10-Tx PC11-Rx
    //GPIO1.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // PB10-Tx PB11-Rx

    GPIO_Init(GPIOC, &GPIO1);
}

void USART_Config(BaudRate)
{
    clearServoReceiveBuffer();
    USART_InitTypeDef USART;
    USART.USART_BaudRate = BaudRate;
    USART.USART_WordLength = USART_WordLength_8b;
    USART.USART_StopBits = USART_StopBits_1;
    USART.USART_Parity = USART_Parity_No;
    USART.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART);
    USART_Cmd(USART3, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStruct;
    // configure the USART3 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init (&NVIC_InitStructure);

	// enable the USART3 receive interrupt
	USART_ITConfig (USART3, USART_IT_RXNE, ENABLE);

    // enable USART3
    USART_Cmd (USART3, ENABLE);
}

void soft_delay(long int ticks)
{
    for(; ticks > 0; ticks-- );
}

long frequency = 9600;

/*void delay(int n){
    int i;
    for(i=0; i<n; i++);
}*/

int main(void)
{
    SystemInit();
    RCC_Config();
    GPIO_Config();

    uint8_t ID_broadcast = 0xFE; // Sends to all Dynamixels
    uint16_t testAngleVal = 0;
    bool flag = 0;


    uint8_t ID_test = 0x02;
    uint16_t testAngleVal1 = 150;
    uint16_t testAngleVal2 = 300;
    uint16_t i = 0;
    //setServoMovingSpeed(254, 200, 0);

 //   setServoCWAngleLimit (254, 0);
  //  setServoCCWAngleLimit (254, 0);
  while(1)
  {
// Uploading standart paramiters
//      for (; frequency <= 1001000; frequency *= 1.01)
//      {
//          USART_Config(frequency);
//          setID(ID_broadcast,  (uint8_t)2);
//          setID(ID_broadcast,  (uint8_t)2);
//          setID(ID_broadcast,  (uint8_t)2);
//          setID(ID_broadcast,  (uint8_t)2);
//          setBaudRate (ID_broadcast, 1000000);
//          setServoAngle(ID_test,100);
//          setServoAngle(ID_test,200);
//          setServoCWAngleLimit (ID_broadcast, (uint16_t) 0);
//          setServoCCWAngleLimit (ID_broadcast, (uint16_t) 1023);
//          setServoReturnDelayMicros (ID_broadcast, (uint16_t) 0xFA);
//      }
//      if (frequency > 1000000) frequency = 5000;


////// Test section
////// Uncomment when done with first section to test
        USART_Config(1000000);
        setID(ID_broadcast,  ID_test);
        setID(ID_broadcast,  ID_test);

        setServoAngle(ID_broadcast, testAngleVal1);
        setServoAngle(ID_broadcast, testAngleVal2);
        setServoAngle(ID_test,testAngleVal );
        setServoAngle(ID_test, testAngleVal2);

        setServoToJointMode(ID_test);
//        setID(ID_broadcast,  ID_test);
//        setID(ID_broadcast,  ID_test);
        setServoAngle(ID_broadcast, testAngleVal1);
        setServoAngle((uint8_t) 1,testAngleVal );


        setServoAngle(ID_test, testAngleVal2);

        setServoToWheelMode(ID_broadcast);
        setServoMovingSpeed(ID_broadcast, (uint16_t)200, 0x0000);
        setServoMovingSpeed(ID_broadcast, (uint16_t)1023, 0x0000);
        setServoMovingSpeed(ID_broadcast, (uint16_t)1200, 0x0400);
        setServoMovingSpeed(ID_broadcast, (uint16_t)2046, 0x0400);
        setServoMovingSpeed(ID_broadcast, (uint16_t)0, 0x0000);

      //  setServoMovingSpeed(254, 300, 0x0000);
      //  setServoAngle(1,250);
//        for(i=20; i<150; i++){

//         setServoAngle(1,i++);
      //  delay(40000);
//         if(i>359)i=0;
     //   setServoAngle(1,200);

//            delay(1000);
//        }
//        delay(10000000);
//        for(i=150; i>20; i--){
//            setServoAngle(254,i);
//            delay(1000);
//        }
//        delay(10000000);

    //    setServoMovingSpeed(254, 300, 1);
//        uint8_t packet[] = {0xFF,0xFF,0xFD,0x00,0x01,0x03,0x00,0x01};
//        uint16_t answer;
//        answer = CRC16_BUYPASS(packet,sizeof(packet));
//        answer=answer;

  }
}

