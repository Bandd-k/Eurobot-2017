#include "Interrupts.h"

#include "gpio.h"
#include "Regulator.h"
#include "Path.h"
#include "Pins.h"
#include <math.h>
#include "usart.h"
#include "robot.h"
#include "board.h"
#include "Manipulators.h"
extern uint32_t ticks;
extern float whole_angle,angle_enc_real;
int indexSpeeds = 0, indexDists = 0;
char traceFlag, movFlag, endFlag, allpointsreached;
double timeofred = 0;
int16_t int_cnt = 0;
static uint8_t prev_val, current_val;
bool start_cylinder_rot;
int starting_time;
float rot_time;


////////////////////////////////////////////////////////////////////////////////
//_________________________________TIMERS_____________________________________//
////////////////////////////////////////////////////////////////////////////////
void TIM2_IRQHandler(void)
{
  //USB_OTG_BSP_TimerIRQ();
}
////////////////////////////////////////////////////////////////////////////////


int numberofrot= 0;
float tempor = 0;

void TIM6_DAC_IRQHandler() // 100Hz  // Рассчет ПИД регуляторов колес, манипулятора и считывание данных сонаров
{
  if (startFlag) {
    stop_cnt++;
  }
  if((stop_cnt - starting_time > fabs(rot_time)) && start_cylinder_rot){
        setServoMovingSpeed(3, (uint16_t)(0), 0x0000);//CCW    // ÂÃÐÓÇÈÒÜ
        setServoMovingSpeed(2, (uint16_t)(0), 0x0000);
        setServoMovingSpeed(3, (uint16_t)(0), 0x0000);//CCW    // ÂÃÐÓÇÈÒÜ
        setServoMovingSpeed(2, (uint16_t)(0), 0x0000);
        start_cylinder_rot = false;
  }
    if (stop_cnt >= 17500){
        curState.pidEnabled = 0;
        char i;
        for (i = 0; i < 4; i++)
        {
            setVoltageMaxon(WHEELS[i], (uint8_t) 1,  (float) 0);
        }
      }
  TIM6->SR = 0;
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  GetDataForRegulators(); // обновление входных данных для ПИД
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);


  if (curState.kinemEn) FunctionalRegulator(&vTargetGlob[0],  &regulatorOut[0]); // рассчет  кинематики и насыщения

    char i = 0;
    for(i; i < 4; i++)
    {
        if (curState.pidEnabled) setSpeedMaxon(WHEELS[i], regulatorOut[i]);
    }
    tempor = encodermagner(tempor);
    if (tempor !=2)
    {
        numberofrot += tempor;
    }
    whole_angle = numberofrot * 360 + getCurrentEncoderAngle();
}
////////////////////////////////////////////////////////////////////////////////

void TIM7_IRQHandler() // 33kHz
{
  TIM7->SR = 0;

////////////////////////////////////////////////////////////////////////////////
}
////////////////////////////////////////////////////////////////////////////////


void TIM8_UP_TIM13_IRQHandler() // рассчет траекторного регулятора
{
   // set_pin(PWM_DIR[8]);

  TIM13->SR = 0;
 NVIC_DisableIRQ(TIM6_DAC_IRQn);  //отключение ПИД на время расчета

float temp = (curPath.phiZad-robotCoord[2]);

 // if (((fabs(curPath.lengthTrace) )) <= fabs(curPath.Coord_local_track[0]) && // достигнута заданная точка по положению и углу
    if (((fabs(curPath.lengthTrace) - fabs(curPath.Coord_local_track[0])) < 0.01) && ((fabs(curPath.Coord_local_track[1])) < 0.01)&& // достигнута заданная точка по положению и углу
        (fabs(rangeAngle(&temp)) < 0.02))
        {
          traceFlag = 1;  // точка достигнута
        }
    else {
          traceFlag = 0;
          allpointsreached = 0;
         }
    if (!movFlag)
        if (points[0].movTask) {
            movFlag=(points[0].movTask)();}
        else {
            movFlag =1;} // действие в процессе движения
    if (traceFlag&&movFlag&&(!endFlag)){
        if (points[0].endTask)
             endFlag = ((char (*)(float))(points[0].endTask))(points[0].endTaskP1);
        else
            endFlag =1;
        }
    // действие в конечной точке
    if (traceFlag && movFlag && endFlag)
        {
          if (lastPoint > 0) //Остались ли точки в стеке
          {
            CreatePath(&points[1], &points[0], &curPath);// задать новый участок
            totalPointComplite++;
            removePoint(&points[0],&lastPoint); //удалить ткущую точку
            endFlag=0;
            movFlag=0;
            traceFlag=0;
            allpointsreached = 0;
          }
          else {
                allpointsreached = 1;
               }
        }


//////////////////////////// COMPUTING SPEEDS /////////////////////////////////

 if (curState.trackEn)
    {
       TrackRegulator(&robotCoord[0],&robotSpeed[0], (&curPath),&vTargetGlob[0]); // расчет глобальных скоростей
    }
NVIC_EnableIRQ(TIM6_DAC_IRQn);
}



////////////////////////////////////////////////////////////////////////////////
//__________________________________EXTI______________________________________//
////////////////////////////////////////////////////////////////////////////////


//#define EXTI2_PIN               pin_id(PORTD,0)         //Разъем EXTI2//
void EXTI0_IRQHandler(void)
{
    static uint32_t lasttick;

  EXTI->PR=0x1;
  char temp = 2;
//  if ( pin_val(EXTI2_PIN) ) temp |=0x80;
//  sendAnswer(0x1E,&temp, 1);
  ticks = ticks;

    timeofred = (ticks - lasttick) ;
    lasttick = ticks;

}

//#define EXTI5_PIN               pin_id(PORTD,1)         //Разъем EXTI5//
void EXTI1_IRQHandler(void)
{
//  static uint32_t lasttick;

  EXTI->PR=0x2;
  char temp = 5;
//  if ( pin_val(EXTI5_PIN) ) temp |=0x80;
//  sendAnswer(0x1E,&temp, 1);

//  timeofred = (ticks - lasttick) ;
//  lasttick = ticks;
}

//#define EXTI4_PIN               pin_id(PORTD,2)         //Разъем EXTI4//
void EXTI2_IRQHandler(void)
{
  EXTI->PR=0x4;
  char temp = 4;
  //if ( pin_val(EXTI4_PIN) ) temp |=0x80;
  //sendAnswer(0x1E,&temp, 1);
}

//#define EXTI6_PIN               pin_id(PORTD,3)         //Разъем EXTI6//
void EXTI3_IRQHandler(void)
{
  //  static uint32_t lasttick;
  EXTI->PR=0x8;
  char temp = 6;
//  if ( pin_val(EXTI6_PIN) ) temp |=0x80;
//  sendAnswer(0x1E,&temp, 1);

  //  timeofred = (ticks - lasttick) ;
  //lasttick= ticks;

}


void EXTI4_IRQHandler(void)
{
  //static uint32_t lasttick;

  EXTI->PR=0x10;
  char temp = 9;
//  if ( pin_val(EXTI9_PIN) ) temp |=0x80;
//  sendAnswer(0x1E,&temp, 1);

//  timeofred = (ticks - lasttick) ;
//  lasttick= ticks;

}

//#define EXTI7_PIN               pin_id(PORTD,6)         //Разъем EXTI7//
//#define EXTI8_PIN               pin_id(PORTD,7)         //Разъем EXTI8//

void EXTI9_5_IRQHandler(void)
{

    current_val = pin_val(EXTI8_PIN);

    if(current_val != prev_val){
        static uint32_t lasttick;
        timeofred = (ticks - lasttick) ;
        lasttick= ticks;
    }
    prev_val = current_val;

/*    if(EXTI->PR&(1<<7)){
        static uint32_t lasttick;

        EXTI->PR&(1<<7);
        char temp = 8;
        if ( pin_val(EXTI8_PIN) ) temp |=0x80;
        sendAnswer(0x1E,&temp, 1);

        timeofred = (ticks - lasttick) ;
        lasttick= ticks;
    }*/
  /*if (EXTI->PR&(1<<6))
  {
    EXTI->PR=(1<<6);
    char temp = 7;
    if ( pin_val(EXTI7_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }
  if (EXTI->PR&(1<<7))
  {
    EXTI->PR=(1<<7);
    char temp = 8;
    if ( pin_val(EXTI8_PIN) ) temp |=0x80;
    sendAnswer(0x1E,&temp, 1);
  }
*/
}


//#define EXTI3_PIN               pin_id(PORTC,12)        //Разъем EXTI3//
//#define EXTI10_PIN              pin_id(PORTC,13)        //Разъем EXTI10//*/
//#define EXTI1_PIN               pin_id(PORTA,15)        //Разъем EXTI1///
void EXTI15_10_IRQHandler(void)
{

  if (EXTI->PR&(1<<12))
  {
    EXTI->PR=(1<<12);
    char temp = 3;
//    if ( pin_val(EXTI3_PIN) ) temp |=0x80;
//    sendAnswer(0x1E,&temp, 1);
  }
  if (EXTI->PR&(1<<13))
  {
    EXTI->PR=(1<<13);
    char temp = 10;
//    if ( pin_val(EXTI10_PIN) ) temp |=0x80;
//    sendAnswer(0x1E,&temp, 1);
  }
  if (EXTI->PR&(1<<15))
  {
      int_cnt++;
    EXTI->PR=(1<<15);
    char temp = 1;
//    if ( pin_val(EXTI1_PIN) ) temp |=0x80;
//    sendAnswer(0x1E,&temp, 1);
  }

}

////////////////////////////////////////////////////////////////////////////////
//___________________________________I2C______________________________________//
////////////////////////////////////////////////////////////////////////////////
void delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//___________________________________ADC______________________________________//
////////////////////////////////////////////////////////////////////////////////
void DMA2_Stream0_IRQHandler(void)
{
DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
}
////////////////////////////////////////////////////////////////////////////////
