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
#include "Board.h"  //���� �������������

#include "gpio.h" // ������ � ������� �����-������
#include "Pins.h" // ����������� ����� �� �����
#include "Interrupts.h"
#include "regulator.h"
// ���������� �����, ����������, �����������

#include "usart.h" //����� � ������������� ��������
#include "robot.h"  //����������� ������������ ������ � ��� �������� �������
#include "Manipulators.h"
// ����� � �����������
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END;

char mode;
float distance[10];
int stop_cnt = 0;
//int flag_kostil = 0 ;
int main(void)
{

    __disable_irq();
    initAll();
    CloseLauncher();

      USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
            USB_OTG_HS_CORE_ID,
#else
            USB_OTG_FS_CORE_ID,
#endif
            &USR_desc,
            &USBD_CDC_cb,
            &USR_cb);

       //����� ��������� ������������� ������� � ������ ����������� �����������


//    __disable_irq();

//    conf_pin(EXTI2_PIN, INPUT, PUSH_PULL, FAST_S, PULL_UP);
__enable_irq();


while(1){
        //throwRightCollectorIntoBox(76);

// if (pin_val(EXTI9_PIN))
//      {
//        flag_kostil  = 1;
//        startFlag = 1;
//      }
//     else if (flag_kostil == 0)
//      {
//        stop_cnt = 0;
//        startFlag = 0;
//      }
        distance[FRONT] = 10 + MAX_DIST + (float)adcData[FRONT]*(MIN_DIST - MAX_DIST)/(MAX_RAW_SENSOR - MIN_RAW_SENSOR);
//        distance[FRONT_RIGHT] = MIN_DIST + (float)MAX_RAW_SENSOR*(MAX_DIST - MIN_DIST)/(MAX_RAW_SENSOR - MIN_RAW_SENSOR) - (MAX_DIST - MIN_DIST)*(float)adcData[FRONT_RIGHT]/(MAX_RAW_SENSOR - MIN_RAW_SENSOR);
//        distance[BACK_LEFT] = MIN_DIST + (float)MAX_RAW_SENSOR*(MAX_DIST - MIN_DIST)/(MAX_RAW_SENSOR - MIN_RAW_SENSOR) - (MAX_DIST - MIN_DIST)*(float)adcData[BACK_LEFT]/(MAX_RAW_SENSOR - MIN_RAW_SENSOR);
        distance[BACK] = 10 + MAX_DIST + (float)adcData[BACK]*(MIN_DIST - MAX_DIST)/(MAX_RAW_SENSOR - MIN_RAW_SENSOR);
   }

}
