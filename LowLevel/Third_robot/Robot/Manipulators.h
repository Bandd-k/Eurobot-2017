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

#define CLOSE_LID_VALUE 0.0401
#define OPEN_LID_VALUE 0.07
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


///////////////////////BALL COLLECTORS////////////////////////////////


#define EXTI_HIGHERSENSOR_RIGHT EXTI4_PIN
#define EXTI_LOWERSENSOR_RIGHT EXTI3_PIN
#define EXTI_HIGHSENSOR_LEFT EXTI8_PIN
#define EXTI_LOWERSENSOR_LEFT EXTI7_PIN
#define DNMXL_MAN_RIGHT 1
#define DNMXL_MAN_LEFT 2
#define EXTI_POLOL1_RIGHT EXTI1_PIN
#define EXTI_POLOL2_RIGHT EXTI2_PIN
#define EXTI_POLOL1_LEFT EXTI5_PIN
#define EXTI_POLOL2_LEFT EXTI6_PIN
//#define DNMXL_ANGLE_MAN_ON 89
//#define DNMXL_ANGLE_MAN_OFF 3
#define DNMXL_ANGLE_MAN_TRANSIT_ON 140

//#define DNMXL_LEFT_ANGLE_MAN_ON 150
//#define DNMXL_LEFT_ANGLE_MAN_OFF 58
//#define DNMXL_ANGLE_MAN_THROW 59
//#define DNMXL_ANGLE_MAN_UP_RIGHT 77
//#define DNMXL_ANGLE_MAN_UP_LEFT 84

bool downRightCollectorToGetBalls(int);
bool upRightCollectorWithBalls(int);
bool throwRightCollectorIntoBox(int);
bool downLeftCoolectorToGetBalls(int);
bool upLeftCollectorWithBalls(int);
bool throwLeftCollectorIntoBox(int);
bool r12();
void polulu_outside_right();
void polulu_outside_left();
//////////////////////////////////////////////////////////////////////

/////////////////////FACE CYLINDER GETTER////////////////////////////
#define DOWN_FACE_CYL 0.03907
#define UP_FACE_CYL 0.01509
#define SRV_FACE_BTN 8
#define SRV_GETTER_FACE_BTN 9
#define OPEN_GETTER_FACE 0.0375
#define CLOSE_GETTER_FACE 0.06
#define TRIP_FACE_CYL 0.02354
#define CORRECTOR_SRV_BTN 7
#define CORRECTOR_OPEN 0.1072
#define CORRECTOR_CLOSE 0.05
void DownFaceCylinder();
void GetFaceCylinder();
//void GoToTripFaceCylinder();
void OpenCyinderCorrector();
void CloseCylinderCorrector();
void OpenFaceCylinderGetter();
void GoBackFaceCylinderManipulator();
void CloseGetterFaceCylinderManipulator();
///////////////////////////////////////////////////////////////////


/////////////////////////BACK CYLINDER GETTER/////////////////////
#define DOWN_BACK_CYL 0.0849
#define UP_BACK_CYL 0.04636
#define SRV_BACK_BTN 5
#define SRV_BACK_GETTER_BTN 6
#define OPEN_GETTER_BACK 0.0375
#define CLOSE_GETTER_BACK 0.0827
#define TRIP_BACK_CYL 0.002354
void DownBackCylinder();
void GetBackCylinder();
//void GoToTripBackCylinder();
void OpenBackCylinderGetter();
void GoBackBackCylinderManipulator();
void CloseGetterBackCylinderManipulator();
///////////////////////////////////////////////////////////////////

//////////////////////////SEESAW CORRECTOR/////////////////////////
#define DNMXL_SEESAW 3
#define DNMXL_SEESAW_PRE_ON 76
#define DNMXL_SEESAW_ON 60
#define DNMXL_SEESAW_OFF 145
void OpenSeesawCorrector();
void CloseSeesawCorrector();

#define LEFTUP 6
#define RIGHTUP 7
#define DOWNONE 8
#define ForwardConstant 0.0909
#define NEITRAL 0.12999
#define REVERSE 0.105999
void PropollersToForward(void);
void PropollersToNeitral(void);
void PropollersToForward(void);
///////////////////////////////////////////////////////////////////

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
