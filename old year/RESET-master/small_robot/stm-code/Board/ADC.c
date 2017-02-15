////////////////////////////////////////////////////////////////////////////////
//_________________________________ADC________________________________________//
////////////////////////////////////////////////////////////////////////////////

#include "adc.h"
#include "math.h"
#include "stm32f4xx.h"
#include "robot.h"





////////////////////////////////////////////////////////////////////////////////
void adcConfig()
{/*
  ADC1->SQR1 |= ADC1_NUMB << 20;
  ADC1->SQR3 |= 0  | (1 << 5)  | (2 << 10)  | (3 << 15) | (8 << 20) | (9 << 25);
  ADC1->SQR2 |= 11| (12 << 5)  | (14 << 10)  | (15 << 15) ;

//  ADC1->SMPR2 |= (7 << 12) | (7 << 15) | (7 << 24) | (7 << 27);
//  ADC1->SMPR1 |= (7 << 3) | (7 << 6) | (7 << 9);

  ADC1->CR1 |= ADC_SCAN_MODE ; //����� ������������. ���������� 12 ���.
  ADC1->CR2 |= ADC_CR2_DMA|ADC_CR2_CONT|ADC_CR2_DDS; //��������� �������������
  //������ DMA. ��� ����� ����������� ������ DMA ����� ������� ��������������.

  DMA2_Stream0->CR |= 0 << 25;        //�������� channel 0
  DMA2_Stream0->PAR |= (uint32_t) &ADC1->DR;//������ ����� ��������� - ������� ���������� �������������� ��� ��� ���������� �������.
  DMA2_Stream0->M0AR |= (uint32_t) &adcData; //������ ����� ������ - ������� ����� ������� � RAM.
  DMA2_Stream0->CR &= ~DMA_SxCR_DIR; //����������� �������� ������ - ������ �� ���������, ������ � ������.
  DMA2_Stream0->NDTR = ADC1_NUMB+1; //���������� ������������ ��������
  DMA2_Stream0->CR &= ~DMA_SxCR_PINC; //����� ��������� �� ���������������� ����� ������ ���������.
  DMA2_Stream0->CR |= DMA_SxCR_MINC|DMA_SxCR_CIRC; //����� ������ ���������������� ����� ������ ���������.
  DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0; //����������� ������ ��������� - 16 ���.
  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0; //����������� ������ ������ - 16 ���
  DMA2_Stream0->CR |= DMA_SxCR_PL; //��������� - ����� ������� (Very High)
  DMA2_Stream0->CR |= DMA_SxCR_TCIE;

  //DMA2->LIFCR = DMA_LIFCR_CTCIF0;
  DMA2_Stream0->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; //��������� ������ ������ 1 DMA
  ADC1->CR2 |= ADC_CR2_ADON; //�������� ���
  ADC1->CR2 |= ADC_CR2_SWSTART;/*/
   ADC1->SQR1 |= ADC1_NUMB << 20;
  ADC1->SQR3 |= 0  | (1 << 5)  | (2 << 10)  | (3 << 15) | (8 << 20) | (9 << 25);
  ADC1->SQR2 |= 11| (12 << 5)  | (14 << 10)  | (15 << 15) ;

  ADC1->SMPR2 |= (7 << 3) | (7 << 6) | (7 << 12) | (7 << 15);
  ADC1->SMPR1 |= (7) | (7 << 3) | (7 << 6) | (7 << 9) | (7 << 24) | (7 << 27);

  ADC1->CR1 |= ADC_SCAN_MODE ; //����� ������������. ���������� 12 ���.
  ADC1->CR2 |= ADC_CR2_DMA|ADC_CR2_CONT|ADC_CR2_DDS; //��������� �������������
  //������ DMA. ��� ����� ����������� ������ DMA ����� ������� ��������������.

  DMA2_Stream0->CR |= 0 << 25;        //�������� channel 0
  DMA2_Stream0->PAR |= (uint32_t) &ADC1->DR;//������ ����� ��������� - ������� ���������� �������������� ��� ��� ���������� �������.
  DMA2_Stream0->M0AR |= (uint32_t) &adcData; //������ ����� ������ - ������� ����� ������� � RAM.
  DMA2_Stream0->CR &= ~DMA_SxCR_DIR; //����������� �������� ������ - ������ �� ���������, ������ � ������.
  DMA2_Stream0->NDTR = ADC1_NUMB+1; //���������� ������������ ��������
  DMA2_Stream0->CR &= ~DMA_SxCR_PINC; //����� ��������� �� ���������������� ����� ������ ���������.
  DMA2_Stream0->CR |= DMA_SxCR_MINC|DMA_SxCR_CIRC; //����� ������ ���������������� ����� ������ ���������.
  DMA2_Stream0->CR |= DMA_SxCR_PSIZE_0; //����������� ������ ��������� - 16 ���.
  DMA2_Stream0->CR |= DMA_SxCR_MSIZE_0; //����������� ������ ������ - 16 ���
  DMA2_Stream0->CR |= DMA_SxCR_PL; //��������� - ����� ������� (Very High)
  DMA2_Stream0->CR |= DMA_SxCR_TCIE;

  //DMA2->LIFCR = DMA_LIFCR_CTCIF0;
  DMA2_Stream0->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE; //��������� ������ ������ 1 DMA
  ADC1->CR2 |= ADC_CR2_ADON; //�������� ���
  ADC1->CR2 |= ADC_CR2_SWSTART;





}
////////////////////////////////////////////////////////////////////////////////
