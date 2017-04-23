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

/*
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	// Set priority
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	// Set sub priority
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	// Enable interrupt
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	// Add to NVIC
	NVIC_Init(&NVIC_InitStruct);*/
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

// 1000000*3 // 2000000*3 // 1000000*2 // 2000000*3 // 4000000

//long int tempora = 1000000*3 ;
//numberofrot = 0;
    while(1)
    {
//
//        if (pin_val (EXTI9_PIN)){
//            startFlag = 1;
//        }
//        else{
//            startFlag = 0;
//        }
//        goOutsideWithSuckingManipulator();
//        softDelay(2000000*3);
//        goInsideWithSuckingManipulator();
//        softDelay(2000000*3);

//        setPositionOfCylinderCarrierByTime(1000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        setPositionOfCylinderCarrierByTime(2000000*3);
//        softDelay(2000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        setPositionOfCylinderCarrierByTime(1000000*2);
//        softDelay(1000000*2);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        setPositionOfCylinderCarrierByTime(2000000*3);
//        softDelay(2000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        softDelay(1000000*3);
//        setPositionOfCylinderCarrierByTime(4000000);
//        softDelay(2000000*3);
//        softDelay(2000000*3);
//        softDelay(2000000*3);
//        softDelay(2000000*3);
//        setPositionOfCylinderCarrierByTime(-(4000000+2000000*3+1000000*2+2000000*3+1000000*3));





    }
}
