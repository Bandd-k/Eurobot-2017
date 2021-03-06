#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Regulator.h"
#include "Board.h"
#include "math.h"


double timeofred;
char color, color_check[8];
float magnetincoderdata[10];
float r,b,R,B;
float whole_angle, values[10], angle_encoder, angle_enc_real, whole_starting_angle, whole_angle_prev, angle_before_movement;
bool direction = true;
bool start_cylinder_rot;

float rot_time;
extern int numberofrot;

manipStateStruct mState = {0,148,0,0} ;


//values[0] = 0;
//values[1] = 0;
//values[2] = 0;
//values[3] = 0;
//values[4] = 0;
//values[5] = 0;
//values[6] = 0;
//values[7] = 0;
//values[8] = 0;
//values[9] = 0;
#define MAXTIME 350
void manip_core()
{
//  if (mState.manipState==MANIP_START)
//  {
//    mState.manipState|=MANIP_GO_IN;
//  }
//  //-GO---IN---------------------------------------------------------
//
//  if (mState.manipState&MANIP_GO_IN)
//  {
//    if (!(mState.manipState&MANIP_IN))
//    {
//        mState.manipState&=~(MANIP_GO_IN|MANIP_GO_OUT|MANIP_MOVE_OUT);
//        mState.manipState=MANIP_MOVE_IN;//mState.manipState|=MANIP_MOVE_IN;
//        setServoTorque(4,1023);
//        setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
//        setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
//
//        mState.starting_time = stop_cnt;
//        mState.button_cnt = 0;
//        mState.manipState&=~MANIP_DIN_READY;
//    } else mState.manipState&=~MANIP_GO_IN;
//  }
//  if (mState.manipState&MANIP_MOVE_IN)
//  {
//    setServoTorque(4,1023);
//    setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
//    if ((stop_cnt - mState.starting_time > 0)&&(!(mState.manipState&MANIP_DIN_READY)))
//    {
//        mState.manipState|=MANIP_DIN_READY;
//        servo_rotate_180(mState.angle);
//        servo_rotate_180(mState.angle);
//    }
//
//
//    if(pin_val(UPPER_SWITCH)){
//            mState.button_cnt++;
//        }
//
//    if ((stop_cnt - mState.starting_time > MAXTIME)||(mState.button_cnt > 5))
//    {
//       setServoTorque(4,200);
//       setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//
//       setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//       mState.manipState&=0;
//       mState.manipState=MANIP_IN;//|=
//
//    }
//  }
//    //---GO--OUT-------------------------------------------------
//
//    if (mState.manipState&MANIP_GO_OUT)
//  {
//    if (!(mState.manipState&MANIP_OUT))
//    {
//        mState.manipState&=~(MANIP_GO_OUT|MANIP_GO_IN|MANIP_MOVE_IN);
//        mState.manipState=MANIP_MOVE_OUT; //mState.manipState|=MANIP_MOVE_OUT;
//        setServoTorque(4,1023);
//        setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);
//        setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);
//        mState.starting_time = stop_cnt;
//        mState.button_cnt = 0;
//    } else mState.manipState&=~MANIP_GO_OUT;
//  }
//  if (mState.manipState&MANIP_MOVE_OUT)
//  {
//        setServoTorque(4,1023);
//        setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);
//        setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);
//    if(pin_val(DOWN_SWITCH)){
//            mState.button_cnt++;
//        }
//    if ((stop_cnt - mState.starting_time > MAXTIME)||(mState.button_cnt > 5 ))
//    {
//
//       setServoTorque(4,200);
//       setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//       setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//       //mState.manipState=0;
//    }
//    if ((stop_cnt - mState.starting_time > MAXTIME)||(mState.button_cnt > 10))
//    {
//       mState.manipState=MANIP_OUT;
//       setServoTorque(4,200);
//       setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//       setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//       //mState.manipState=0;
//
//    }
//
//
//
//  }
}

void softDelay(__IO unsigned long int ticks)
{
    for(; ticks > 0; ticks--);
}

void lowerCylinderRGBManipulator(){
    setPWM((char)RGB_Cylinder, (float)OPEN_RGB_Cylinder);
}

void liftCylinderRGBManipulator(){
    setPWM((char)RGB_Cylinder, (float)CLOSE_RGB_Cylinder);
}

bool rotateColoredCylinder(){
    set_pin(INPUT3_CONTROL);
    set_pin(INPUT3_CONTROL);
    reset_pin(INPUT4_CONTROL);
    reset_pin(INPUT4_CONTROL);
    lowerCylinderRGBManipulator();
}

bool stopRotateColoredCylinder(){
    reset_pin(INPUT3_CONTROL);
    reset_pin(INPUT3_CONTROL);
}


bool goInsideWithSuckingManipulator(int angle){
    setServoTorque(4,1023);
    setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);

    setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);

    softDelay(6000000/2);

    servo_rotate_180(angle);
    int starting_time = stop_cnt;
    int button_cnt = 0;
    while(stop_cnt - starting_time < MAXTIME){
        if (button_cnt<50){
            setServoTorque(4,1000);
            setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
        }
        if(pin_val(UPPER_SWITCH)){
            button_cnt++;
        }
        if(button_cnt > 50){
            setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
            setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);

        }
        if (button_cnt > 100){
                break;
        }
    }

    setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
    setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
}

//bool goInsideWithSuckingManipulator(int angle){
//    mState.angle = angle;
//    mState.manipState|=MANIP_GO_IN;
//}
/*
//    setVoltage(BTN_SUCKING_MANIPULATOR-1, -1);
//
    setServoMovingSpeed(4, (uint16_t)(850+1024), 0x0400);
    setServoMovingSpeed(4, (uint16_t)(850+1024), 0x0400);

    int starting_time = stop_cnt;
    int button_cnt = 0;

    softDelay(4800000/2);
//
//    setVoltage(BTN_SUCKING_MANIPULATOR-1, 0);
//
//    setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
    servo_rotate_180(angle);
//
//    setVoltage(BTN_SUCKING_MANIPULATOR-1, -1);
//
//    setServoMovingSpeed(4, (uint16_t)(1024+650), 0x0400);
//

    while(stop_cnt - starting_time < 300){
        if(pin_val(UPPER_SWITCH)){
            button_cnt++;
        }
        if(button_cnt > 100){
            break;
        }
    }
    setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//
//    setVoltage(BTN_SUCKING_MANIPULATOR-1, 0);
//    setServoAngle(4,200);
//    servo_rotate_90(angle);
//    setServoAngle(4,0);
}
*/
/*bool goInsideButDifferentRotate(int angle){
    set_pin(INPUT1_CONTROL);
    set_pin(INPUT1_CONTROL);
    reset_pin(INPUT2_CONTROL);
    reset_pin(INPUT2_CONTROL);

    softDelay(6000000/2);

    reset_pin(INPUT1_CONTROL);
    reset_pin(INPUT1_CONTROL);

    servo_rotate_90(angle);

    set_pin(INPUT1_CONTROL);
    set_pin(INPUT1_CONTROL);

    int starting_time = stop_cnt;
    int button_cnt = 0;
    while(stop_cnt - starting_time < 300){
        if(pin_val(UPPER_SWITCH)){
            button_cnt++;
        }
        if(button_cnt > 100){
            break;
        }
    }

    reset_pin(INPUT1_CONTROL);
    reset_pin(INPUT1_CONTROL);
}*/
//bool goInsideButDifferentRotate(int angle){
//    setServoTorque(4,1000);
//    setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
//
//    setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
//    softDelay(6000000/2);
//    servo_rotate_180(angle);
//    int starting_time = stop_cnt;
//    int button_cnt = 0;
//    while(stop_cnt - starting_time < MAXTIME){
//        if (button_cnt<50){
//            setServoTorque(4,1000);
//            setServoMovingSpeed(4, (uint16_t)(1000+1024), 0x0400);
//
//        }
//        if(button_cnt > 50){
//            setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
//            setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
//        }
//        if(pin_val(UPPER_SWITCH)){
//            button_cnt++;
//        }
//        if(button_cnt > 100){
//            break;
//        }
//    }
//    setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
//    setServoMovingSpeed(4, (uint16_t)(1024), 0x0400);
//}



/*
//    setVoltage(BTN_SUCKING_MANIPULATOR-1, -1);
//
    setServoMovingSpeed(4, (uint16_t)(850+1024), 0x0400);
    int starting_time = stop_cnt;
    int button_cnt = 0;

    softDelay(4800000/2);



//    setServoMovingSpeed(4, (uint16_t)(0), 0x0000);



//    setVoltage(BTN_SUCKING_MANIPULATOR-1, 0);
    servo_rotate_90(angle);



//    setServoMovingSpeed(4, (uint16_t)(650+1024), 0x0400);




//    setVoltage(BTN_SUCKING_MANIPULATOR-1, -1);

    while(stop_cnt - starting_time < 300){
        if(pin_val(UPPER_SWITCH)){
            button_cnt++;
        }
        if(button_cnt > 100){
            break;
        }
    }
    setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
//    setVoltage(BTN_SUCKING_MANIPULATOR-1, 0);

}
*/

bool goOutsideWithSuckingManipulator(){
    setServoTorque(4,1000);
    setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);

    setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);

    int starting_time = stop_cnt;
    int button_cnt = 0;
    while((stop_cnt - starting_time) < MAXTIME){ //300
        if (button_cnt<50){
            setServoTorque(4,1000);
            setServoMovingSpeed(4, (uint16_t)(1000), 0x0000);
        }
        if(pin_val(DOWN_SWITCH)){
            button_cnt++;
        }
        if(button_cnt > 50){
            setServoMovingSpeed(4, (uint16_t)(0), 0x0000);

            setServoMovingSpeed(4, (uint16_t)(0), 0x0000);

        }
        if(button_cnt > 100){
            break;
        }
    }

    setServoMovingSpeed(4, (uint16_t)(000), 0x0000);

    setServoMovingSpeed(4, (uint16_t)(000), 0x0000);
    setServoTorque(4,100);
}
//
//bool goOutsideWithSuckingManipulator(){
//
//    mState.angle = angle;
//    mState.manipState|=MANIP_GO_OUT;
//
//}
/*
    setServoMovingSpeed(4, (uint16_t)(900), 0x0000);//CCW    // ��������


    //setVoltage(BTN_SUCKING_MANIPULATOR-1, 1);

    int starting_time = stop_cnt;
    int button_cnt = 0;
    while((stop_cnt - starting_time) < 300){ //300
        if(pin_val(DOWN_SWITCH)){
            button_cnt++;
        }
        if(button_cnt > 100){
            break;
        }
    }
    setServoMovingSpeed(4, (uint16_t)(0), 0x0000);
    //setVoltage(BTN_SUCKING_MANIPULATOR-1, 0);
}*/


/*  this commented function is wrong, so don't check it and don't use it
void GetDataForManipulator(void)
{
    int i = 3000000;
    CubesCatcherAngle = adcData[(char)CUBES_CATCHER_ADC - 1] / 36 * 3.3;//*360/3.3

    while(--i > 0);

    float sum=0;
    for(i=0;i<9;i++){
        values[i] = values[i+1];
        sum+=values[i];
    }
    sum += values[9];
    sum/=10;

    if(( previous_status < 380  &&  previous_status >= 270 ) && ( sum < 90  &&  sum >= 0 ))
         number_full_rotations++;

    if((previous_status >= 0  &&  previous_status < 90 ) && ( sum < 380 && sum >= 270 ))
        number_full_rotations--;

//    if(fabs(CubesCatcherAngle - previous_status)>100)
//        number_full_rotations--;



    values[9] = CubesCatcherAngle;

    whole_angle = number_full_rotations*360 + CubesCatcherAngle;

    previous_status = sum;
}*/

char getCurrentColor(){
    int z = 1000;
    while(z>0){
        z--;
        reset_pin(EXTI2_PIN);//red, S3
        int j,i = 15000;
            for(; i> 0; i--);
        r = timeofred;
        R = 10000./(r);
        i = 15000;
            for(; i> 0; i--);

        set_pin(EXTI2_PIN);//blue

        i = 15000;
            for(; i> 0; i--);
        b = timeofred;
        B = 10000./(b);
////
        if(R > B  &&  R/B > 1.7)color = 'Y';
        else if(B > R  &&  B/R > 1.6) color = 'B';
        else color = 'W';

        softDelay(500);
        for(j=0;j<7;j++)color_check[j] = color_check[j+1]; //filter
        color_check[7] = color;

        for(j=0;j<8;j++)if(color_check[i] != color){color='n';break;}

        if(color!='n') break;
    }

    return color;
}

float getCurrentEncoderAngle(void){

    angle_encoder = adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096.0;
    angle_enc_real = 32*pow(angle_encoder, 0.41) - 27./360 * angle_encoder + 30 - 65*exp(-pow((angle_encoder-55)/60, 2)) + 25*exp(-pow((angle_encoder-117)/33, 2));
    angle_enc_real  = filtrating(&magnetincoderdata[0],angle_enc_real,10);
    //whole_angle = numberofrot * 360 + angle_enc_real;
    return angle_enc_real;
}

float filtrating(float* array_to_filtrate, float new_data, int len_of_array){
    int i;
    float sum=0;
    for (i=0;i<len_of_array-1;i++){
            array_to_filtrate[i]=array_to_filtrate[i+1];
    }
    array_to_filtrate[len_of_array-1] = new_data;
    for (i=0;i<len_of_array;i++){
        sum += array_to_filtrate[i];
    }
    sum = sum/len_of_array;
    return sum;
}



void setCurrentAngleAsBeginning(void){
    whole_angle = getCurrentEncoderAngle();
    whole_starting_angle = getCurrentEncoderAngle();
}

void setPositionOfCylinderCarrier(float desiredAngle){

//    angle_encoder = adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096.0;
//    angle_enc_real = 32*pow(angle_encoder, 0.41) - 27./360 * angle_encoder + 30 - 65*exp(-pow((angle_encoder-55)/60, 2)) + 25*exp(-pow((angle_encoder-117)/33, 2));
//    whole_angle = numberofrot * 360 + angle_enc_real;
    angle_before_movement = whole_angle;

    if(desiredAngle < whole_angle){

        while(fabs(desiredAngle - whole_angle) > 30 || (desiredAngle < whole_angle)){

//            angle_encoder = adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096.0;
//            angle_enc_real = 32*pow(angle_encoder, 0.41) - 27./360 * angle_encoder + 30 - 65*exp(-pow((angle_encoder-55)/60, 2)) + 25*exp(-pow((angle_encoder-117)/33, 2));
//            whole_angle = numberofrot * 360 + angle_enc_real;

            setServoMovingSpeed(3, (uint16_t)(720), 0x0000);//CCW    // ��������
            setServoMovingSpeed(2, (uint16_t)(850 + 1024), 0x0400);//CW  ��������

//            setServoMovingSpeed(3, (uint16_t)(800 + 1024), 0x0400);//CW
//            setServoMovingSpeed(2, (uint16_t)(810), 0x0000);//CCW


//            if((int)fabs((angle_enc_real)) % 360 > 353){
//                softDelay(800000);
//                angle_encoder = adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096.0;
//                angle_enc_real = 32*pow(angle_encoder, 0.41) - 27./360 * angle_encoder + 30 - 65*exp(-pow((angle_encoder-55)/60, 2)) + 25*exp(-pow((angle_encoder-117)/33, 2));
//                whole_angle = numberofrot * 360 + angle_enc_real;
//            }

            if(fabs(angle_before_movement - whole_angle) > fabs(desiredAngle - angle_before_movement)){//-10 at the end
                setServoMovingSpeed(3, (uint16_t)0, 0x0000);
                setServoMovingSpeed(2, (uint16_t)0, 0x0000);
//                whole_angle += 2*fabs(angle_before_movement - whole_angle);
                break;
            }
            whole_angle_prev = whole_angle - 20;
        }

    }

    else if(desiredAngle > whole_angle){

        whole_angle_prev = whole_angle + 10;

        while(fabs(desiredAngle - whole_angle) > 30 || (desiredAngle > whole_angle)){

//            angle_encoder = adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096.0;
//            angle_enc_real = 32*pow(angle_encoder, 0.41) - 27./360 * angle_encoder + 30 - 65*exp(-pow((angle_encoder-55)/60, 2)) + 25*exp(-pow((angle_encoder-117)/33, 2));
//            whole_angle = numberofrot * 360 + angle_enc_real;

            setServoMovingSpeed(3, (uint16_t)(827 + 1024), 0x0400);//CW ���������
            setServoMovingSpeed(2, (uint16_t)(860), 0x0000);//CCW  ���������

//            setServoMovingSpeed(3, (uint16_t)(710), 0x0000);//CCW
//            setServoMovingSpeed(2, (uint16_t)(815 + 1024), 0x0400);//CW


//            if((int)fabs(angle_enc_real) % 360 > 353){
//                softDelay(600000);
//                angle_encoder = adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096.0;
//                angle_enc_real = 32*pow(angle_encoder, 0.41) - 27./360 * angle_encoder + 30 - 65*exp(-pow((angle_encoder-55)/60, 2)) + 25*exp(-pow((angle_encoder-117)/33, 2));
//                whole_angle = numberofrot * 360 + angle_enc_real;
//            }

            if(/*(fabs(desiredAngle - whole_angle) > fabs(desiredAngle - whole_angle_prev)) && */fabs(angle_before_movement - whole_angle) > fabs(desiredAngle - angle_before_movement)){//-10 at the end
                setServoMovingSpeed(3, (uint16_t)0, 0x0000);
                setServoMovingSpeed(2, (uint16_t)0, 0x0000);
//                whole_angle -= 2*fabs(angle_before_movement - whole_angle);
                break;
            }

            whole_angle_prev = whole_angle + 20;
        }

    }

    setServoMovingSpeed(2, (uint16_t)0, 0x0000);
    setServoMovingSpeed(3, (uint16_t)0, 0x0000);

}

void setPositionOfCylinderCarrierByTime(float time){
    // 3 is left, 2 is right; from front
    rot_time = time * 200;
    start_cylinder_rot = true;
    float coef_left=1.23*1.1;
    float coef_right=1.15*1.07*1.13;
    starting_time = stop_cnt;
    if (time > 0){
        // direction CCW = 0x0000, CW = 0x0400
        setServoMovingSpeed(3, (uint16_t)(720*coef_left), 0x0000);
        setServoMovingSpeed(2, (uint16_t)(720*coef_right + 1024), 0x0400);
        setServoMovingSpeed(3, (uint16_t)(720*coef_left), 0x0000);
        setServoMovingSpeed(2, (uint16_t)(720*coef_right + 1024), 0x0400);

    }
    else{
        coef_left=1.23;
        coef_right=1.17;
        setServoMovingSpeed(3, (uint16_t)(827*coef_left + 1024), 0x0400);//CW ���������
        setServoMovingSpeed(2, (uint16_t)(750*coef_right), 0x0000);//CCW  ���������
        setServoMovingSpeed(3, (uint16_t)(827*coef_left + 1024), 0x0400);//CW ���������
        setServoMovingSpeed(2, (uint16_t)(750*coef_right), 0x0000);//CCW  ���������

    }


//    time *= 1000;
//    if (time>0){
//             time= time*1.00; //experimental difference in speed
//             setServoMovingSpeed(3, (uint16_t)(720*1.2), 0x0000);//CCW    // ��������
//             setServoMovingSpeed(2, (uint16_t)(730*1.2 + 1024), 0x0400);//CW  ��������
//             softDelay(time);
//
//             setServoMovingSpeed(3, (uint16_t)(0), 0x0000);//CCW    // ��������
//             setServoMovingSpeed(2, (uint16_t)(0), 0x0000);
//             setServoMovingSpeed(3, (uint16_t)(0), 0x0000);//CCW    // ��������
//             setServoMovingSpeed(2, (uint16_t)(0), 0x0000);
//    }
//    else {
//
//            setServoMovingSpeed(3, (uint16_t)(827*1.2 + 1024), 0x0400);//CW ���������
//            setServoMovingSpeed(2, (uint16_t)(750*1.2), 0x0000);//CCW  ���������
//            softDelay(-time);
//            setServoMovingSpeed(3, (uint16_t)(0), 0x0000);//CCW    // ��������
//            setServoMovingSpeed(2, (uint16_t)(0), 0x0000);
//            setServoMovingSpeed(3, (uint16_t)(0), 0x0000);//CCW    // ��������
//            setServoMovingSpeed(2, (uint16_t)(0), 0x0000);
//    }
}

void increaseByGivenAngle(float givenAngle){
    setPositionOfCylinderCarrier(whole_angle - givenAngle);
}



void decreaseByGivenAngle(float givenAngle){
    setPositionOfCylinderCarrier(whole_angle + givenAngle);
}


void servo_elevate_in()
{
    setServoAngle((uint8_t)SERVO_ELEVATE, (uint16_t) SERVO_ELEVATE_IN);
}

void servo_elevate_out()
{
    setServoAngle((uint8_t)SERVO_ELEVATE, (uint16_t) SERVO_ELEVATE_OUT);
}

void servo_rotate_90(int angl)
{
    setServoAngle((uint8_t)SERVO_ROTATE, (uint16_t) angl);
}

void servo_rotate_180(int angl)
{
    setServoAngle((uint8_t)SERVO_ROTATE, (uint16_t) angl);
}

//
//
//float HaveCylinder = 0;
//void DetectorCylinder()
//{
//    if (pin_val(DETECT_CYLINDER))
//    {
//        HaveCylinder = 1;
//    }
//}


float CubesCatcherAngle = 0;
float prevCubesCatcherAngle = 0;
float diff;
float arCubesCatcherAngle[10];

float encodermagner(float prevencodermagner){

        arCubesCatcherAngle[9] = angle_enc_real;//adcData[(char)CUBES_CATCHER_ADC - 1] *360 / 4096;//*360/3.3
        int i = 0;
        float smoothed = 0;
        for (i;i<9;i++) {
                arCubesCatcherAngle[i] = arCubesCatcherAngle[i+1];
                smoothed+= arCubesCatcherAngle[i];
        }
        smoothed /= 10;


        if ((arCubesCatcherAngle[0] - arCubesCatcherAngle[9]) > 100) {
                diff = 1;
                //softDelay(100000);
        }
        else if ((arCubesCatcherAngle[0]- arCubesCatcherAngle[9]) < -100) {
                diff = -1;
                //softDelay(100000);
        }
        else diff = 0;
        prevCubesCatcherAngle = smoothed;
        if ((diff != 0)  && (prevencodermagner !=0)){
            diff =2;}
        return diff;
}



bool switchOnPneumo()
{
//    set_pin(PIN1_12V);
    set_pin(PIN2_12V);
//    set_pin(PIN3_12V);
//    set_pin(PIN4_12V);
//    set_pin(PIN5_12V);
//    set_pin(PIN6_12V);
    return 0;
}

bool switchOffPneumo()
{
//    reset_pin(PIN1_12V);
    reset_pin(PIN2_12V);
//    reset_pin(PIN3_12V);
//    reset_pin(PIN4_12V);
//    reset_pin(PIN5_12V);
//    reset_pin(PIN6_12V);
    return 0;
}


///////////////////////////////////////////////////////////////
