#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_

#include <stdbool.h>
#include <stdint.h>
#include "pins.h"

void softDelay(unsigned long int);

///////////////////////////TOWER BUILDER///////////////////////
#define ID_FRONT 11                     // Front manipulator
#define ID_BACK 11                      // Back manipulator
#define CLOSED_ANG 0
#define OPEN_ANG 300

bool open_tower(int8_t);
bool close_tower(int8_t);
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////
#define ID_RIGHT 18                     // If to look on robot from the front
#define ID_LEFT 1
#define OPEN_ANG_RIGHT 300
#define OPEN_ANG_LEFT 0
#define CLOSED_ANG_RIGHT 150
#define CLOSED_ANG_LEFT 150
#define ONE_CUBE_CATCHED_ANGLE 30      // angle defining difference in manipulators angles in 1 cube is caught
#define TWO_CUBES_CATCHED_ANGLE 90     // angle defining difference in manipulators angles in 2 cubes are caught
#define CUBES_CATCHER_ADC 1
#define CUBES_CATCHER_MOTOR_CH 5

extern uint16_t l[10];

bool openCubesCatcher();
bool closeCubesCatcher(uint8_t*);
void initCubeCatcherPID(void);
void GetDataForManipulator(void);
void pidLowLevelManipulator(float, float);

//////////////////////////FUNNY ACTION IN EUROBOT 2017/////////////////////////////////////

#define CLOSE_LID_VALUE 0.0735
#define OPEN_LID_VALUE 0.0895
#define FUNNY_ACTION_BTN_CHANNEL 4

void OpenLauncher();
void CloseLauncher();

///////////////////////////PNEUMO//////////////////////////////

bool pneumoIn();
bool pneumoOut();
bool switchOnPneumo();
bool switchOffPneumo();
///////////////////////////////////////////////////////////////
////////////////small robot///////////////////

#define SEASHEL_ID  9
#define DOORS_ID  2
#define STARTINGPOS 60
#define ENDINGPOS  240
#define DORS_OPENPOS 300
#define DOORS_CLOSEDPOS 50



void liftSeashell_up();
void liftSeashell_down();
void close_dors();
void stop_dors();
void open_dors();

#define CH_FISHIN_GSERVO  5
#define ID_FISHING_MANIPULATOR 3
#define  ANG_OPEN_FISHING_MANIPULATOR  140
#define ANG_SUPEROPEN_FISHING_MANIPULATOR 133
#define  ANG_CLOSE_FISHING_MANIPULATOR  270
#define  ANG_HALF_CLOSE_FISHING_MANIPULATOR 200
#define  DUTY_FISH_CATCH 0.08
#define TIMEOPENDORS 10000000
#define TIMECloseDors 19000000

#define  DUTY_FISH_UNCATCH  0.03 //0.022
#define  FISHING_MANIPULATOR_TORQUE  1000


#define OPENEDSEASHELANGLE 280
#define CLOSEDSEASHELANGLE 80


void OpenFishingManipulator();
void CloseFishingManipulator();
void HalfOpenFishingManipulator();
void TearFish(); //присоеденить
void UnTearFish(); //otсоеденить

void servo_elevate_in();
void servo_elevate_out();
void servo_rotate_90();
void servo_rotate_180();


#define SERVO_ELEVATE  1 //defining servos
#define SERVO_ROTATE 2

#define SERVO_ELEVATE_IN 0 //defining angular values
#define SERVO_ELEVATE_OUT 155
#define SERVO_ROTATE_90  60
#define SERVO_ROTATE_180  150


//////////////////////////SEESAW CORRECTORS/////////////////////////
#define DNMXL_SEESAW_RIGHT 2
#define DNMXL_SEESAW_LEFT 4
#define DNMXL_SEESAW_PRE_ON 76
#define DNMXL_SEESAW_RIGHT_ON 100 //NEED TO CONFIGURATE!!!!
#define DNMXL_SEESAW_RIGHT_OFF 240 //NEED TO CONFIGURATE!!!!
#define DNMXL_SEESAW_LEFT_ON 197 //NEED TO CONFIGURATE!!!!
#define DNMXL_SEESAW_LEFT_OFF 60 //NEED TO CONFIGURATE!!!!
void OpenRihgtSeesawCorrector();
void CloseRightSeesawCorrector();
void OpenLeftSeesawCorrector();
void CloseLeftSeesawCorrector();

///////////////////////////////////////////////////////////////////

//////////////////////////////DOOR/////////////////////////////////
#define DNMXL_DOOR 3
#define OPEN_DOOR 72 //NEED TO CONFIGURATE!!!!
#define CLOSE_DOOR 224 // NEED TO CONFIGURATE!!!
void OpenDoor();
void CloseDoor();
////////////////////////////////////////////////////////////////////

/////////////////////////COOLERS////////////////////////////////////
#define LEFTUP 5
#define RIGHTUP 6
#define DOWNONE 7
#define FORWARD 0.0909
#define NEUTRAL 0.105999
#define REVERSE 0.12999
void PropollersToForward(void);
void PropollersToNeitral(void);
void PropollersToReverse(void);
///////////////////////////////////////////////////////////////////


/////////////////////////ROTATING MANIPULATOR///////////////////////
#define DNMXL_ROTATING 1
void SwitchOnRotating();
void SwitchOffRotating();
////////////////////////////////////////////////////////////////////
/////////////////small robot eurobot 2017 for manipulators

/*#define UPPER_SWITCH EXTI1_PIN
#define DOWN_SWITCH EXTI2_PIN
#define INPUT2_CONTROL EXTI3_PIN
#define INPUT1_CONTROL EXTI4_PIN


bool goUpWithSuckingManipulator();
bool goDownWithSuckingManipulator();

*/
extern int stop_cnt;

#endif
