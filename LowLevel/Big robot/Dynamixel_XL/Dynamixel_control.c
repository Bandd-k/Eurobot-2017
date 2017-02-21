/*

Code to control multiple Dynamixel AX-12 servo motors over USART
on an STM32F4 chip.

*/

#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "Dynamixel_control.h"

__asm__(".global _printf_float");
__asm__(".global _scanf_float");

uint8_t servoErrorCode = 0;

ServoResponse response;

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;

typedef enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3,
//    RESETing = 6
} ServoCommand;

#define ID                  0x03
#define BAUD_RATE           0x04
#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define CURRENT_LOAD        0x28
#define MOVING_SPEED        0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24
#define CW_ANGLE_LIMIT      0x06
#define CCW_ANGLE_LIMIT     0x08



uint16_t CRC16_BUYPASS(uint8_t *data, size_t len)
{
  uint16_t crc = 0x0000;
  size_t j;
  int i;
  for (j=len; j>0; j--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (i=0; i<8; i++) {
      if (crc & 0x8000) crc = (crc<<1) ^ 0x8005;
      else crc <<= 1;
    }
  }
  return (crc);
}

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params)
{
    uint8_t packet[100] = {};
    uint8_t packet_size = 0;

    sendServoByte (0xff); // command header
    packet[packet_size] = 0xff;
    packet_size++;

    sendServoByte (0xff);
    packet[packet_size] = 0xff;
    packet_size++; // command header

    sendServoByte (0xfd);
    packet[packet_size] = 0xfd;
    packet_size++; // command header

    sendServoByte (0x00); // empty byte
    packet[packet_size] = 0x00;
    packet_size++;

    sendServoByte (servoId);
    packet[packet_size] = servoId;
    packet_size++;  // servo ID
    //uint8_t checksum = servoId;

    uint16_t length = numParams + 3;
    uint8_t length_l = (length & 0x00FF);
    uint8_t length_h = (length>>8) & 0x00FF;
    sendServoByte (length_l);
    packet[packet_size] = length_l;
    packet_size++;
    sendServoByte (length_h);
    packet[packet_size] = length_h;
    packet_size++;
      // number of following bytes
    sendServoByte ((uint8_t)commandByte);  // command
    packet[packet_size] = (uint8_t)commandByte;
    packet_size++;
    //checksum += numParams + 3 + commandByte;

    uint8_t i = 0;
    for (; i < numParams; i++)
    {
        sendServoByte (params[i]);  // parameters
        packet[packet_size] = params[i];
        packet_size++;
    }

    uint16_t crc = CRC16_BUYPASS(packet, packet_size);
    uint8_t crc_l = (crc & 0x00FF);
    uint8_t crc_h = (crc>>8) & 0x00FF;
    sendServoByte (crc_l);
    sendServoByte (crc_h);  // checksum
}

bool getServoResponse (void)
{
    uint16_t retries = 0;

    clearServoReceiveBuffer();

   while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries at start\n");
            #endif
            return false;
        }
    }
    retries = 0;

    response.first_byte = getServoByte();  // servo header (two 0xff bytes)
    response.second_byte = getServoByte();
    response.third_byte = getServoByte();

    response.reserved = getServoByte();

    response.id = getServoByte();
    uint8_t length_l = getServoByte();
    uint8_t length_h = getServoByte();

    response.length = 0x0000;
    response.length += length_l;
    response.length += (uint16_t)(length_h<<8);

    if (response.length > SERVO_MAX_PARAMS)
    {
        #ifdef SERVO_DEBUG
        printf ("Response length too big: %d\n", (int)response.length);
        #endif
        return false;
    }

    while (getServoBytesAvailable() < response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries waiting for params, got %d of %d params\n", getServoBytesAvailable(), response.length);
            #endif
            return false;
        }
    }

    response.error = getServoByte();
    servoErrorCode = response.error;

    uint16_t i = 0;
    for (; i < response.length - 3; i++)
        response.params[i] = getServoByte();

    uint16_t calcChecksum = CRC16_BUYPASS(&response, sizeof(response)-2);
//    i = 0;
//    for (; i < response.length - 3; i++)
//        calcChecksum += response.params[i];
//    calcChecksum = ~calcChecksum;
    uint16_t chc;
    const uint8_t checksum_l = getServoByte();
    const uint8_t checksum_h = getServoByte();
    response.checksum = checksum_l + (uint16_t)(checksum_h << 8);
    chc = response.checksum;
    if (calcChecksum != response.checksum)
    {
        #ifdef SERVO_DEBUG
        printf ("Checksum mismatch: %x calculated, %x received\n", calcChecksum, response.checksum);
        #endif
        return false;
    }

    return true;
}

inline bool getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Servo %d did not respond correctly or at all\n", (int)servoId);
        #endif
        return false;
    }

    if (response.id != servoId)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response ID %d does not match command ID %d\n", (int)response.id);
        #endif
        return false;
    }

    if (response.error != 0)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response error code was nonzero (%d)\n", (int)response.error);
        #endif
        return false;
    }

    return true;
}

// ping a servo, returns true if we get back the expected values
bool pingServo (const uint8_t servoId)
{
    sendServoCommand (servoId, PING, 0, 0);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}

// set an ID to a servo, returns true if we get back the expected values

/////////////////////////////////////////////////////////////////////////////////////////////////                NOT DONE
bool setID (const uint8_t servoId, uint8_t newID)
{
    if (newID > 253)
        return false;

    const uint8_t params[2] = {ID, newID & 0xff};

    sendServoCommand (servoId, WRITE, 2, params);

//    if (!getAndCheckResponse (newID))
//        return false;

    return true;
}

// set a baud rate to a servo, returns true if we get back the expected values

/////////////////////////////////////////////////////////////////////////////////////////////////                NOT DONE
bool setBaudRate (const uint8_t servoId, uint8_t baudRate)
{
    if (BAUD_RATE > 253)
        return false;

    const uint8_t params[2] = {BAUD_RATE, baudRate & 0xff};

    sendServoCommand (servoId, WRITE, 2, params);

//    if (!getAndCheckResponse (servoId))
  //      return false;

    return true;
}


// set the number of microseconds the servo waits before returning a response
// servo factory default value is 500, but we probably want it to be 0
// max value: 510


//////////////////////////////////////////////////////   changed
bool setServoReturnDelayMicros (const uint8_t servoId,
                                const uint16_t micros)
{
    if (micros > 510)
        return false;

    const uint8_t params[4] = {RETURN_DELAY, (uint8_t)0x00,
                               (uint8_t)((micros / 2) & 0xff), (uint8_t)0x00,};

    sendServoCommand (servoId, WRITE, 4, params);

//    if (!getAndCheckResponse (servoId))
  //      return false;

    return true;
}

// set the events that will cause the servo to blink its LED


//////////////////////////////////////////////////////   changed
bool setServoBlinkConditions (const uint8_t servoId,
                              const uint8_t flags)
{
    const uint8_t params[4] = {BLINK_CONDITIONS,(uint8_t)0x00,
                               flags, (uint8_t)0x00};

    sendServoCommand (servoId, WRITE, 4, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}
/*
//(re)set default (factory) parameters except ID and Baudrate

bool setDefault(const uint8_t servoId)
{
    const uint8_t params[1] = {0x02};

    sendServoCommand (servoId, RESETing, 1, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}
*/
// set the events that will cause the servo to shut off torque

//////////////////////////////////////////////////////   changed
bool setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t flags)
{
    const uint8_t params[4] = {SHUTDOWN_CONDITIONS, (uint8_t)0x00,
                               flags, (uint8_t)0x00};

    sendServoCommand (servoId, WRITE, 4, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}


// valid torque values are from 0 (free running) to 1023 (max)

//////////////////////////////////////////////////////   changed
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue)
{
    const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);

    if (torqueValue > 1023)
        return false;

    const uint8_t params[6] = {TORQUE, (uint8_t)0x00,
                               lowByte,
                               highByte, (uint8_t)0x00, (uint8_t)0x00};

    sendServoCommand (servoId, WRITE, 6, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}

//get maximum allowed torque value
bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE, 2};  // read two bytes, starting at address TORQUE

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
        return false;

    *torqueValue = response.params[1];
    *torqueValue <<= 8;
    *torqueValue |= response.params[0];


    return true;
}

// speed values go from 1 (incredibly slow) to 1023 CCW
// speed values go from 1024 (incredibly slow) to 2047 CW
//  0 < speedValue < 1023
// direction CCW = 0x0000, CW = 0x0400
// a value of zero will disable velocity control


//////////////////////////////////////////////////////   changed
bool setServoMovingSpeed (const uint8_t servoId,
                       const uint16_t speedValue, const uint16_t direction)
{
    uint16_t speed = speedValue;
    speed |= direction;
    const uint8_t highByte = (uint8_t)((speed >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(speed & 0xff);

    if (speedValue > 1023)
        return false;

    const uint8_t params[6] = {MOVING_SPEED, (uint8_t)0x00,
                               lowByte,
                               highByte, (uint8_t)0x00, (uint8_t)0x00 };

    sendServoCommand (servoId, WRITE, 6, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}

// get maximum allowed speed
bool getServoMovingSpeed (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {MOVING_SPEED,
                               2};  // read two bytes, starting at address MOVING_SPEED

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
        return false;

    *speedValue = response.params[1];
    *speedValue <<= 8;
    *speedValue |= response.params[0];

    return true;
}

// get current speed
bool getServoCurrentSpeed (const uint8_t servoId,
                              int16_t *speedValue)
{
    const uint8_t params[2] = {CURRENT_SPEED,
                               2};  // read two bytes, starting at address CURRENT_SPEED

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
        return false;

    *speedValue = response.params[1];
    *speedValue <<= 8;
    *speedValue |= response.params[0];

    return true;
}

// get current load on the shaft
bool getCurrentLoad (const uint8_t servoId,
                     uint16_t *loadValue)
{
    const uint8_t params[2] = {CURRENT_LOAD, 2};  // read two bytes, starting at address CURRENT_LOAD

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
        return false;

    *loadValue = response.params[1];
    *loadValue <<= 8;
    *loadValue |= response.params[0];
    *loadValue &= 0x3ff;

    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle ( const uint8_t servoId,
                     const uint16_t angle)
{
    if (angle < 0 || angle > 300)
        return false;

    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));

    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);

    const uint8_t params[6] = {GOAL_ANGLE, (uint8_t)0x00,
                               lowByte,
                               highByte, (uint8_t)0x00, (uint8_t)0x00};

    sendServoCommand (servoId, WRITE, 6, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}

// get current servo angle
bool getServoAngle (const uint8_t servoId,
                    float *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE

    sendServoCommand (servoId, READ, 2, params);

    if (!getAndCheckResponse (servoId))
        return false;

    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];

    *angle = (float)angleValue * 300.0 / 1023.0;

    return true;
}

// set CW angle  limit
// if both CW and CCW angle limits set to 0, endless turn is activated

//////////////////////////////////////////////////////   changed
bool setServoCWAngleLimit (const uint8_t servoId,
                     const uint16_t limitValue)
{
    const uint8_t highByte = (uint8_t)((limitValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(limitValue & 0xff);

    if (limitValue > 1023)
        return false;

    const uint8_t params[6] = {CW_ANGLE_LIMIT, (uint8_t)0x00,
                               lowByte,
                               highByte, (uint8_t)0x00, (uint8_t)0x00};

    sendServoCommand (servoId, WRITE, 6, params);

    if (!getAndCheckResponse (servoId))
        return false;

    return true;
}

// set CCW angle  limit form 0 to 1023
// if both CW and CCW angle limits set to 0, endless turn is activated

//////////////////////////////////////////////////////   changed
bool setServoCCWAngleLimit (const uint8_t servoId,
                     const uint16_t limitValue)
{
    const uint8_t highByte = (uint8_t)((limitValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(limitValue & 0xff);

    if (limitValue > 1023)
        return false;

    const uint8_t params[6] = {CCW_ANGLE_LIMIT, (uint8_t)0x00,
                               lowByte,
                               highByte, (uint8_t)0x00, (uint8_t)0x00};

    sendServoCommand (servoId, WRITE, 6, params);

 //   if (!getAndCheckResponse (servoId))
   //     return false;

    return true;
}

void initServoUSART (void)
{
    // enable the USART3 clock
    RCC_APB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd (RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStruct;

    clearServoReceiveBuffer();

    // set USART3 Tx (PB8) as alternate function open-drain
    GPIO_InitStruct.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init (GPIOB, &GPIO_InitStruct);

	// connect the output pin to the peripheral's alt function
    GPIO_PinAFConfig (GPIOB, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig (GPIOB, GPIO_PinSource9, GPIO_AF_USART3);


    // set up USART3
    USART_InitStructure.USART_BaudRate = 1000000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init (USART3, &USART_InitStructure);

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

void sendServoByte (const uint8_t byte)
{
	  USART_SendData (USART3, (uint16_t)byte);

	  //Loop until the end of transmission
	  while (USART_GetFlagStatus (USART3, USART_FLAG_TC) == RESET);
}

void clearServoReceiveBuffer (void)
{
    receiveBufferStart = receiveBufferEnd;
}

size_t getServoBytesAvailable (void)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;

    if (end >= start)
        return (size_t)(end - start);
    else
        return (size_t)(REC_BUFFER_LEN - (start - end));
}

uint8_t getServoByte (void)
{
    receiveBufferStart++;
    if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
        receiveBufferStart = receiveBuffer;

    return *receiveBufferStart;
}

void USART3_IRQHandler (void)
{
	// check if the USART3 receive interrupt flag was set
	if (USART_GetITStatus (USART3, USART_IT_RXNE))
	{   //USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		const uint8_t byte = (uint8_t)USART_ReceiveData (USART3); // grab the byte from the data register

        receiveBufferEnd++;
        if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
            receiveBufferEnd = receiveBuffer;

        *receiveBufferEnd = byte;
	}

}

