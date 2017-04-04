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
#include "Board.h"

#include "gpio.h"
#include "Pins.h"
#include "Interrupts.h"
#include "Regulator.h"
#include "usart.h"
#include "Robot.h"
#include "Manipulators.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"
#include "stm32fxxx_it.h"
#include <math.h>

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

uint32_t ticks; // global "time" for mesuring frequency of rbg signal
char color, color_check[8]; // for rgb sensor
float r,b,R,B; //for rgb sensor
//extern char lastPoint ;
extern double timeofred;
float ADC_8, ADC_7, ADC_6, ADC_5;
int flag1 =0;
float beginning_angle;
void SysTick_Handler(void)
{
    ticks++;
//  /* Information panel */
////  LCD_SetTextColor(Green);
// // LCD_SetTextColor(LCD_LOG_DEFAULT_COLOR);
}

//#ATTENTION: IN INITALL DISABLED DELAY INHIBIT; IN REGULATOR


int main(void)
{

    __disable_irq();
    initAll();


    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	// Set priority
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	// Set sub priority
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	// Enable interrupt
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	// Add to NVIC
	NVIC_Init(&NVIC_InitStruct);
    SysTick_Config(840);

    USBD_Init(&USB_OTG_dev,
    #ifdef USE_USB_OTG_HS
                USB_OTG_HS_CORE_ID,
    #else
                USB_OTG_FS_CORE_ID,
    #endif
                &USR_desc,
                &USBD_CDC_cb,
                &USR_cb);

    __enable_irq();


//    set_pin(EXTI2_PIN);
//    reset_pin(EXTI1_PIN);
//    set_pin(EXTI7_PIN); // LED to PD7
    //uint8_t ID_test = 2;
  //  float ADC_8, ADC_7, ADC_6, ADC_5;

    while(1)
    {

       /* ADC_8 = pin_val(GENERAL_PIN_7);
        ADC_7 = pin_val(GENERAL_PIN_6);
        ADC_6 = pin_val(GENERAL_PIN_5);
        ADC_5 = pin_val(GENERAL_PIN_4);*/
//        goInsideWithSuckingManipulator();
//        goOutsideWithSuckingManipulator();
        //servo_rotate_180();
//        setPositionOfCylinderCarrier(0.0);
//        servo_rotate_90();
//        setCurrentAngleAsBeginning();
 //       increaseByGivenAngle(100.0);

//        color = getCurrentColor();

//        goOutsideWithSuckingManipulator();
   //     setCurrentAngleAsBeginning();
     //   setCurrentAngleAsBeginning();

   //     increaseByGivenAngle(PREPARE_FOR_SECOND_CYLINDER);
     /*
        goOutsideWithSuckingManipulator();

        increaseByGivenAngle(LIFT_ALL);
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(10000000);
        servo_rotate_180();
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_SECOND_CYLINDER);


        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(10000000);
        servo_rotate_90();


        switchOffPneumo();
        servo_rotate_180();
*/
        setCurrentAngleAsBeginning();
        servo_rotate_90();
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(5000000);
        servo_rotate_180();
        softDelay(5000000);
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_CYLINDER);
        softDelay(1000000);
        switchOffPneumo();
        softDelay(1000000);
        increaseByGivenAngle(STORE_CYLINDER);
        servo_rotate_90();

        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(5000000);
        servo_rotate_180();
        softDelay(5000000);
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_CYLINDER);
        softDelay(1000000);
        switchOffPneumo();
        softDelay(1000000);
        increaseByGivenAngle(STORE_CYLINDER);
        servo_rotate_90();

        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(5000000);
        servo_rotate_180();
        softDelay(5000000);
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_CYLINDER);
        softDelay(1000000);
        switchOffPneumo();
        softDelay(1000000);
        increaseByGivenAngle(STORE_CYLINDER);
        servo_rotate_90();

        decreaseByGivenAngle((LIFT_CYLINDER + STORE_CYLINDER));
        softDelay(5000000);
        decreaseByGivenAngle((LIFT_CYLINDER + STORE_CYLINDER));
        softDelay(5000000);
        decreaseByGivenAngle((LIFT_CYLINDER + STORE_CYLINDER));
        softDelay(5000000);


//        setPositionOfCylinderCarrier(whole_starting_angle);
//        softDelay(5000000);
      /*  decreaseByGivenAngle(STORE_CYLINDER);
        softDelay(5000000);

        decreaseByGivenAngle(LIFT_CYLINDER);
        softDelay(5000000);
        decreaseByGivenAngle(STORE_CYLINDER);
        softDelay(5000000);
/*
        servo_rotate_90();
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(5000000);
        servo_rotate_180();
        softDelay(5000000);
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_CYLINDER);
        switchOffPneumo();
        increaseByGivenAngle(STORE_CYLINDER);
        servo_rotate_90();

        servo_rotate_90();
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(5000000);
        servo_rotate_180();
        softDelay(5000000);
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_CYLINDER);
        switchOffPneumo();
        increaseByGivenAngle(STORE_CYLINDER);
        servo_rotate_90();

        decreaseByGivenAngle(LIFT_CYLINDER);
        softDelay(5000000);
        decreaseByGivenAngle(STORE_CYLINDER);
        softDelay(5000000);
        decreaseByGivenAngle(LIFT_CYLINDER);
        softDelay(5000000);
        decreaseByGivenAngle(STORE_CYLINDER);
        softDelay(5000000);
        decreaseByGivenAngle(LIFT_CYLINDER);
        softDelay(5000000);
        decreaseByGivenAngle(STORE_CYLINDER);
        softDelay(5000000);


      /*  increaseByGivenAngle(PREPARE_FOR_SECOND_CYLINDER);
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(10000000);
        servo_rotate_90();
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_SECOND_CYLINDER);
        switchOffPneumo();
        servo_rotate_180();


        increaseByGivenAngle(PREPARE_FOR_THIRD_CYLINDER);
        goOutsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(10000000);
        servo_rotate_90();
        goInsideWithSuckingManipulator();
        increaseByGivenAngle(LIFT_THIRD_CYLINDER);
        switchOffPneumo();
        servo_rotate_180();
        increaseByGivenAngle(LIFT_ALL);


        goInsideWithSuckingManipulator();
        switchOnPneumo();
        softDelay(10000000);
        servo_rotate_90();


    while(1){
//    increaseByGivenAngle(LIFT_CYLINDER);
//    increaseByGivenAngle(STORE_CYLINDER);


            //setCurrentAngleAsBeginning();
*/
    }
}
