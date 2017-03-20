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

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

uint32_t ticks;
char color, color_check[8];
float r,b,R,B;

extern double timeofred;

void SysTick_Handler(void)
{
    ticks++;
//  /* Information panel */
////  LCD_SetTextColor(Green);
// // LCD_SetTextColor(LCD_LOG_DEFAULT_COLOR);
}

//#ATTENTION: IN INITALL DISABLED DELAY INHIBIT; IN REGULATOR
char t;

int main(void)
{

    __disable_irq();
    initAll();

    /*NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
	 Set priority
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	 Set sub priority
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	 Enable interrupt
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	 Add to NVIC
	NVIC_Init(&NVIC_InitStruct);
    SysTick_Config(840);*/

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

    while(1)
    {
//        /*setDefault((uint8_t)1);
//        setID ((uint8_t)1, (uint8_t)3);
//        setServoToJointMode(3);
/*        setServoAngle((uint8_t)3, (uint16_t)290);
        setServoAngle((uint8_t)3, (uint16_t)120);*/
//        setServoToWheelMode((uint8_t)3);

        setPositionOfCylinderCarrier(200.0);



//        setServoToWheelMode(3);
  /*      setServoMovingSpeed(ID_test, (uint16_t)200, 0x0000);
        setServoMovingSpeed(ID_test, (uint16_t)1023, 0x0000);
        setServoMovingSpeed(ID_test, (uint16_t)1200, 0x0400);
        setServoMovingSpeed(ID_test, (uint16_t)2046, 0x0400);
        setServoMovingSpeed(ID_test, (uint16_t)0, 0x0000);*/
        //
//        goDownWithSuckingManipulator();
//        switchOnPneumo();
//        servo_rotate_90();
//        goUpWithSuckingManipulator();
//        switchOffPneumo();
//        servo_rotate_180();


       }
}
