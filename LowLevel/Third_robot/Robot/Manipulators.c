#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Regulator.h"
#include "Board.h"
#include "Interrupts.h"
void softDelay(__IO unsigned long int ticks)
{
    for(; ticks > 0; ticks--);
}

///////////////////////////TOWER BUILDER///////////////////////

bool openTower(int8_t ID)               // Открыть манипулятор
{
    setServoAngle(ID, OPEN_ANG);
    return 0;
}

bool closeTower(int8_t ID)              // Закрыть манипулятор
{
    setServoAngle(ID, CLOSED_ANG);
    return 0;
}
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////

PidStruct cubesCatcherPID;

float right_servo_angle = 0;
float left_servo_angle = 0;
float prev_right_servo_angle = 1;
float prev_left_servo_angle = 1;

float CubesCatcherAngle;

bool openCubesCatcher()
{
    setServoAngle((uint8_t)ID_RIGHT, (uint16_t)OPEN_ANG_RIGHT);
    setServoAngle((uint8_t)ID_LEFT, (uint16_t)OPEN_ANG_LEFT);

    return 0;
}

bool closeCubesCatcher(uint8_t *numberOfCubesCatched)
{
    setServoReturnDelayMicros((uint8_t)ID_RIGHT, (uint16_t) 0);
    setServoReturnDelayMicros((uint8_t)ID_LEFT, (uint16_t) 0);
    setServoTorque((uint8_t)ID_RIGHT, 500);
    setServoTorque((uint8_t)ID_LEFT, 500);
    setServoAngle((uint8_t)ID_RIGHT, (uint16_t)CLOSED_ANG_RIGHT);
    setServoAngle((uint8_t)ID_LEFT, (uint16_t)CLOSED_ANG_LEFT);
    softDelay(20000000);
    while((prev_right_servo_angle != right_servo_angle) || (prev_left_servo_angle != left_servo_angle))
    {
        prev_right_servo_angle = right_servo_angle;
        prev_left_servo_angle = left_servo_angle;
        getServoAngle((uint8_t)ID_RIGHT, &right_servo_angle);
        softDelay(1000);
        getServoAngle((uint8_t)ID_LEFT, &left_servo_angle);
    }
    float difference = right_servo_angle - left_servo_angle;
    if ((difference > 1) && (difference <= 15))
    {
        *numberOfCubesCatched = 1;
    }
    else if ((difference > 15))
    {
        *numberOfCubesCatched = 2;
    }
    else
        *numberOfCubesCatched = 0;              // no cubes were caught or number of cubes is unknown

    right_servo_angle = 0;
    left_servo_angle = 0;
    prev_right_servo_angle = 1;
    prev_left_servo_angle = 1;

    return 0;
}

void initCubeCatcherPID(void)
{
  	cubesCatcherPID.p_k = 5.00;
  	cubesCatcherPID.i_k = 0.0;
  	cubesCatcherPID.d_k = 0.0;
  	cubesCatcherPID.pid_on = 1;
  	cubesCatcherPID.pid_error_end  = 3;
  	cubesCatcherPID.pid_output_end = 1000;
  	cubesCatcherPID.max_sum_error =16.0;
  	cubesCatcherPID.max_output = 1;
  	cubesCatcherPID.min_output = 0.01;
}

void GetDataForManipulator(void)
{
  CubesCatcherAngle = adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;//*360/3.3



}

void pidLowLevelManipulator(float targetAngle, float currentAngle) //вычисление ПИД регулятора манипулятора
{
    cubesCatcherPID.target = targetAngle;//
    cubesCatcherPID.current = currentAngle; // current manipulator's position
    pidCalc(&cubesCatcherPID);
    setVoltage((char)CUBES_CATCHER_MOTOR_CH, cubesCatcherPID.output);
}
///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////

bool pneumoIn()
{

}

bool pneumoOut()
{

}


void liftSeashell_up()
{
        setServoTorque(SEASHEL_ID , 500);
        setServoCWAngleLimit(SEASHEL_ID , 0);
        setServoCCWAngleLimit(SEASHEL_ID , 1023);
        setServoAngle(SEASHEL_ID , STARTINGPOS);
        softDelay(20000000);
        /*
        setServoAngle(ID, endingPos);
        soft_delay(20000000);
        soft_delay(20000000);
        setServoAngle(ID, startingPos);*/
        }

void liftSeashell_down()
{
    setServoTorque(SEASHEL_ID , 500);
    setServoCWAngleLimit(SEASHEL_ID , 0);
    setServoCCWAngleLimit(SEASHEL_ID , 1023);
    setServoAngle(SEASHEL_ID , ENDINGPOS);
    softDelay(20000000);
}
void close_dors()
{
    setServoTorque(DOORS_ID , 500);

    //setServoAngle(Dors_ID, doors_closedPos );
    //soft_delay(20000000);
    setServoMovingSpeed(DOORS_ID,1000,0x0400);
    softDelay(20000000);

}
void stop_dors()
{
    setServoMovingSpeed(DOORS_ID,0,0x0400);
}

void open_dors()
{
    setServoTorque(DOORS_ID , 1000);
    //setServoAngle(Dors_ID, doors_closedPos );
    //soft_delay(20000000);
    setServoMovingSpeed(DOORS_ID,1000,0x000);
    softDelay(9000000);
    setServoMovingSpeed(DOORS_ID,0,0x000);
}

void OpenFishingManipulator()
{
    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)FISHING_MANIPULATOR_TORQUE);
    setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_OPEN_FISHING_MANIPULATOR); // открыто - ловить1!!
}

void DeepOpenFishingManipulator()
{
    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)FISHING_MANIPULATOR_TORQUE);
    setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_SUPEROPEN_FISHING_MANIPULATOR); // открыто - ловить1!!
}


void OpenLauncher(){
    setPWM((char)FUNNY_ACTION_BTN_CHANNEL, (float)OPEN_LID_VALUE);
}

void CloseLauncher(){
    setPWM((char)FUNNY_ACTION_BTN_CHANNEL, (float)CLOSE_LID_VALUE);
}

void CloseFishingManipulator()
{
     setServoTorque((uint8_t)ID_FISHING_MANIPULATOR,(uint16_t) FISHING_MANIPULATOR_TORQUE);
     setServoAngle((uint8_t)ID_FISHING_MANIPULATOR,(uint16_t) ANG_CLOSE_FISHING_MANIPULATOR);  // закрыто - спрятать
}

void HalfOpenFishingManipulator()
{
    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t) FISHING_MANIPULATOR_TORQUE);
    setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_HALF_CLOSE_FISHING_MANIPULATOR);  // полузакрыто - тащить!
}

void TearFish()
{
    setVoltage((char)CH_FISHIN_GSERVO - 1,(float) -DUTY_FISH_CATCH);
    softDelay(9000000);
    setVoltage((char)CH_FISHIN_GSERVO - 1, (float) -DUTY_FISH_CATCH+0.005);


}

void UnTearFish()
{   setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_HALF_CLOSE_FISHING_MANIPULATOR);
    setVoltage((char)CH_FISHIN_GSERVO - 1, (float) -DUTY_FISH_UNCATCH); //0
    softDelay(9000000);
    setVoltage((char)CH_FISHIN_GSERVO - 1, (float) -DUTY_FISH_UNCATCH-0.005);
    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t) 850);
    uint16_t ang1 = 10;
    int i=0;
    while ( i<5)
    {
        setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_HALF_CLOSE_FISHING_MANIPULATOR+ang1);
        softDelay(900000);
        setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_HALF_CLOSE_FISHING_MANIPULATOR-ang1);
        softDelay(900000);
        ++i;
    }
    setServoTorque((uint8_t)ID_FISHING_MANIPULATOR,(uint16_t) FISHING_MANIPULATOR_TORQUE);
    setServoAngle((uint8_t)ID_FISHING_MANIPULATOR, (uint16_t)ANG_HALF_CLOSE_FISHING_MANIPULATOR);

}

void Open_seashell_doors()
{
   setServoTorque(DOORS_ID , 1000);
    //setServoAngle(Dors_ID, doors_closedPos );
    //soft_delay(20000000);
    setServoCWAngleLimit(DOORS_ID,(uint16_t) 0);
    setServoCCWAngleLimit(DOORS_ID,(uint16_t) 1023);
    setServoAngle(DOORS_ID,OPENEDSEASHELANGLE);


}



void close_seashell_doors()
{
      setServoTorque(DOORS_ID , 1000);
    //setServoAngle(Dors_ID, doors_closedPos );
    //soft_delay(20000000);
    setServoCWAngleLimit(DOORS_ID,(uint16_t) 0);
    setServoCCWAngleLimit(DOORS_ID,(uint16_t) 1023);
    setServoAngle(DOORS_ID,CLOSEDSEASHELANGLE);

}

void servo_elevate_in()
{
    setServoAngle((uint8_t)SERVO_ELEVATE, (uint16_t) SERVO_ELEVATE_IN);
}
void servo_elevate_out()
{
    setServoAngle((uint8_t)SERVO_ELEVATE, (uint16_t) SERVO_ELEVATE_OUT);
}
void servo_rotate_90()
{
    setServoAngle((uint8_t)SERVO_ROTATE, (uint16_t) SERVO_ROTATE_90);
}
void servo_rotate_180()
{
    setServoAngle((uint8_t)SERVO_ROTATE, (uint16_t) SERVO_ROTATE_180);
}
bool switchOnPneumo()
{
    set_pin(PIN1_12V);
    set_pin(PIN2_12V);
    set_pin(PIN3_12V);
    set_pin(PIN4_12V);
    set_pin(PIN5_12V);
    set_pin(PIN6_12V);
    return 0;
}

bool switchOffPneumo()
{
    reset_pin(PIN1_12V);
    reset_pin(PIN2_12V);
    reset_pin(PIN3_12V);
    reset_pin(PIN4_12V);
    reset_pin(PIN5_12V);
    reset_pin(PIN6_12V);
    return 0;
}



////set_pin(broj pina)
////reset_pin(broj pina)
////pin_val(vrijednost pina testira), ima vrijednost - 1, nema vrijednost -0

///////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////
////////////////////////BALL COLLECTORS/////////////////////////
///////////////////////////////////////////////////////////////


/////////////////////////////RIGHT BALL COLLECTOR/////////////////////
bool downRightCollectorToGetBalls(int angle_down)//, int speed)
{
//    setServoTorque(DNMXL_MAN_RIGHT,200);
//    setServoAngle(DNMXL_MAN_RIGHT,DNMXL_ANGLE_MAN_TRANSIT_ON);
    setServoTorque(DNMXL_MAN_RIGHT,200);
    setServoAngle(DNMXL_MAN_RIGHT,angle_down);
        //setVoltage(5,1);
        //set_pin(EXTI_SERVOPOLOL1);
        //reset_pin(EXTI_SERVOPOLOL2);
        //while ((pin_val (EXTI_HIGHERSENSOR)))
        //{
          //  setVoltage(5,0);
          //reset_pin(EXTI_SERVOPOLOL1);
        //}
}

bool upRightCollectorWithBalls(int angle_up)//, int speed)
{
    setServoTorque(DNMXL_MAN_RIGHT,700);
   setServoAngle(DNMXL_MAN_RIGHT,angle_up);
}

void polulu_outside_right (){
      set_pin(EXTI_POLOL1_RIGHT);
         reset_pin(EXTI_POLOL2_RIGHT);
         while (!(pin_val (EXTI_HIGHERSENSOR_RIGHT)) && (stop_cnt <=8900) )
        {
            ;
        }//setVoltage(5,0);
             reset_pin(EXTI_POLOL2_RIGHT);
             reset_pin(EXTI_POLOL1_RIGHT);

}

bool throwRightCollectorIntoBox(int angle)
{
         setServoTorque(DNMXL_MAN_RIGHT,127);
//        float throwAnggle;
         setServoAngle(DNMXL_MAN_RIGHT,angle);
//         getServoAngle(DNMXL_MAN_RIGHT,&throwAnggle);

         /*while (throwAnggle=DNMXL_ANGLE_MAN_THROW)
         {
             ;
         }*/
         //setVoltage(5,-1);
         softDelay(8780000);
         set_pin(EXTI_POLOL2_RIGHT);
         reset_pin(EXTI_POLOL1_RIGHT);
         while(!(pin_val (EXTI_LOWERSENSOR_RIGHT)) && (stop_cnt <=8900))
         {
            ;
          } //setVoltage(5,0);
             reset_pin(EXTI_POLOL2_RIGHT);
             reset_pin(EXTI_POLOL1_RIGHT);

         //setVoltage(5,1);
         set_pin(EXTI_POLOL1_RIGHT);
         reset_pin(EXTI_POLOL2_RIGHT);
         while (!(pin_val (EXTI_HIGHERSENSOR_RIGHT)) && (stop_cnt <=8900))
        {
            ;
        }//setVoltage(5,0);
             reset_pin(EXTI_POLOL2_RIGHT);
             reset_pin(EXTI_POLOL1_RIGHT);
}
bool r12()
{

   set_pin(EXTI_POLOL1_RIGHT);
reset_pin(EXTI_POLOL2_RIGHT);
}

/////////////LEFT BALL COLLECTOR//////////////////////////////////////
bool downLeftCoolectorToGetBalls(int angle_down)//, int speed)
{

    setServoTorque(DNMXL_MAN_LEFT,200);
    setServoAngle(DNMXL_MAN_LEFT,angle_down);
}

bool upLeftCollectorWithBalls(int angle_up)//, int speed)
{
   setServoTorque(DNMXL_MAN_LEFT,700);
   setServoAngle(DNMXL_MAN_LEFT,angle_up);


}

void  polulu_outside_left (){
     set_pin(EXTI_POLOL1_LEFT);
         reset_pin(EXTI_POLOL2_LEFT);
         while (!(pin_val (EXTI_HIGHSENSOR_LEFT)) && (stop_cnt <= 8900))
        {
            ;
        }//setVoltage(5,0);
             reset_pin(EXTI_POLOL2_LEFT);
             reset_pin(EXTI_POLOL1_LEFT);
}

bool throwLeftCollectorIntoBox(int angle)
{
         setServoTorque(DNMXL_MAN_LEFT,127);
       // float throwAnggle;
         setServoAngle(DNMXL_MAN_LEFT,angle);
         //getServoAngle(DNMXL_MAN_LEFT,&throwAnggle);

         /*while (throwAnggle=DNMXL_ANGLE_MAN_THROW)
         {
             ;
         }*/
         //setVoltage(5,-1);
         softDelay(8780000);
         set_pin(EXTI_POLOL2_LEFT);
         reset_pin(EXTI_POLOL1_LEFT);
         while(!(pin_val (EXTI_LOWERSENSOR_LEFT)) && (stop_cnt <=8900))
         {
            ;
          } //setVoltage(5,0);
             reset_pin(EXTI_POLOL2_LEFT);
             reset_pin(EXTI_POLOL1_LEFT);

         //setVoltage(5,1);
         set_pin(EXTI_POLOL1_LEFT);
         reset_pin(EXTI_POLOL2_LEFT);
         while (!(pin_val (EXTI_HIGHSENSOR_LEFT)) && (stop_cnt <=8900))
        {
            ;
        }//setVoltage(5,0);
             reset_pin(EXTI_POLOL2_LEFT);
             reset_pin(EXTI_POLOL1_LEFT);

}
bool r14()
{

   set_pin(EXTI_POLOL1_LEFT);
reset_pin(EXTI_POLOL2_LEFT);
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////FACE CYLINDER MANIPULATOR///////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void GetFaceCylinder()
{
    setPWM((char)SRV_GETTER_FACE_BTN,(float)OPEN_GETTER_FACE);
    softDelay(6400000);
    setPWM((char)SRV_FACE_BTN,(float)DOWN_FACE_CYL);

}
void DownFaceCylinder()
{
    setPWM((char)SRV_FACE_BTN,(float)DOWN_FACE_CYL);
    softDelay(6200000);
    setPWM((char)SRV_GETTER_FACE_BTN,(float)OPEN_GETTER_FACE);
    softDelay(6400000);
    setPWM((char)CORRECTOR_SRV_BTN,(float)CORRECTOR_CLOSE);
    softDelay(6400000);
    setPWM((char)CORRECTOR_SRV_BTN,(float)CORRECTOR_OPEN);
    setPWM((char)SRV_FACE_BTN,(float)UP_FACE_CYL);
}

/*void GoToTripFaceCylinder()
{
   setPWM((char)SRV_GETTER_FACE_BTN,(float)CLOSE_GETTER_FACE);
   softDelay(6400000);
   setPWM((char)SRV_FACE_BTN,(float)TRIP_FACE_CYL);
}*/

void OpenCyinderCorrector()
{
    setPWM((char)CORRECTOR_SRV_BTN,(float)CORRECTOR_OPEN);
}
void CloseCylinderCorrector()
{
    setPWM((char)CORRECTOR_SRV_BTN,(float)CORRECTOR_CLOSE);
}
void OpenFaceCylinderGetter()
{
    setPWM((char)SRV_GETTER_FACE_BTN,(float)OPEN_GETTER_FACE);
}
void GoBackFaceCylinderManipulator()
{
    setPWM((char)SRV_GETTER_FACE_BTN,(float)CLOSE_GETTER_FACE);
    softDelay(6400000);
    setPWM((char)SRV_FACE_BTN,(float)UP_FACE_CYL);
}
void CloseGetterFaceCylinderManipulator()
{
   setPWM((char)SRV_GETTER_FACE_BTN,(float)CLOSE_GETTER_FACE);
}
////////////////////////////////////////////////////////////////////


///////////////////////BACK CYLINDER MANIPULATOR////////////////////
void GetBackCylinder(){
    setPWM((char)SRV_BACK_GETTER_BTN,(float)OPEN_GETTER_BACK);
    softDelay(6200000);
    setPWM((char)SRV_BACK_BTN,(float)DOWN_BACK_CYL);
}
void DownBackCylinder()
{
    setPWM((char)SRV_BACK_BTN,(float)DOWN_BACK_CYL);
    softDelay(6200000);
    setPWM((char)SRV_BACK_GETTER_BTN,(float)OPEN_GETTER_BACK);
    softDelay(6400000);
    setPWM((char)CORRECTOR_SRV_BTN,(float)CORRECTOR_CLOSE);
    softDelay(6400000);
    setPWM((char)CORRECTOR_SRV_BTN,(float)CORRECTOR_OPEN);
    setPWM((char)SRV_BACK_BTN,(float)UP_BACK_CYL);

}

/*void GoToTripBackCylinder()
{
   setPWM((char)SRV_BACK_GETTER_BTN,(float)CLOSE_GETTER_BACK);
   softDelay(6400000);
   setPWM((char)SRV_BACK_BTN,(float)TRIP_BACK_CYL);
}*/
void OpenBackCylinderGetter()
{
    setPWM((char)SRV_BACK_GETTER_BTN,(float)OPEN_GETTER_BACK);
}
void GoBackBackCylinderManipulator()
{
    setPWM((char)SRV_BACK_GETTER_BTN,(float)CLOSE_GETTER_BACK);
    softDelay(6400000);
    setPWM((char)SRV_BACK_BTN,(float)UP_BACK_CYL);

}
void CloseGetterBackCylinderManipulator()
{
    setPWM((char)SRV_BACK_GETTER_BTN,(float)CLOSE_GETTER_BACK);
}
//////////////////////////////////////////////////////////////////////

///////////////////////////SEESAW CORRECOR/////////////////////////////
void OpenSeesawCorrector()
{
    setServoAngle(DNMXL_SEESAW,DNMXL_SEESAW_ON);
}
void CloseSeesawCorrector()
{
    setServoAngle(DNMXL_SEESAW,DNMXL_SEESAW_OFF);
}
///////////////////////////////////////////////////////////////////////
