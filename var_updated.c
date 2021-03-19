#ifndef __VAR__
#define __VAR__

/* -------------------------
Fixed Global Variables
---------------------------*/
//Dimensions in cm
float XMIN = 0.0;
float YMIN = 0.0;
float XMAX = 120.0;
float YMAX = 240.0;
float distanceThreshold = 30.0;

/* -------------------------
Robot Odometry
---------------------------*/
typedef struct{
	float X;       	//in cm
	float Y;        //in cm
	float compass_heading; //in degrees
	float heading;  //wrt to x-axis, in degrees
	float w_left;   //in rad/s
	float w_right;  //in rad/s
	float v_X;      //in cm/s
	float v_Y;      //in cm/s
	float v;        //in cm/s
	float w;        //in rad/s
}Odom;
Odom odom;

/* -------------------------
Robot Structure
---------------------------*/
typedef struct{
	float length;
	float width;
	float wheel_radius;
}Body;
Body body;

/* -------------------------
Sensor Reading States
---------------------------*/
//Power Switch Logic State
int Power_Switch_ON = 0;
int Power_Switch_OFF = 1;
//Ball Limit Switch Logic State
int Ball_Limit_Switch_CONTACT = 0;
int Ball_Limit_Switch_NO_CONTACT = 1;
//Back Limit Switches Logic State
int Back_Limit_Switch_CONTACT = 0;
int Back_Limit_Switch_NO_CONTACT = 1;

/* -------------------------
Functional States
---------------------------*/
int ball_found = 0; //if ball is detected by IR sensor
int ball_collected = 0;	//number of balls in collection area
int ball_caught = 0; //if ball is detected by ball limit switch
int roll_forward = 1;
int boundary = 0;   //0: bottom, 1: left, 2: top, 3: right
int phase = 0; 		//0: move to 2/3, 1: subsequent search
int counter = 0;
/* -------------------------
Timing
---------------------------*/
int ball_caught_t;   //to track when the ball is caught and reverse roller for 1.5 seconds
int moving_back_t;   //just for testing
float count_period_left, count_period_right;	//to determine angular speed of both wheels (used in encoder.c)

/* -------------------------
Functions
---------------------------*/

/* Returns readings from respective IR Sharp Sensors
Sharp 1 (10-80): Best range: 15-50cm (+- 2cm) distance = 27.534 * pow(volt , -1.207)
Sharp 2 (10-80) : Best range: 10-55cm (+- 2cm) distance = 25.1 * pow(volt , -0.904)
Sharp 3 (10-80): Best range: 10-30cm (+- 2cm) distance = 27.534 * pow(volt , -1.207)
Sharp 4 (4-30): Best range: 4-20cm (+- 1cm) distance = 15.02 * pow(volt1 , -1.286) */
int leftSensorReading(){
	//Sharp 2
	float volt = (float)SensorValue[leftIRSensor]/4096*5;
	float distance = 25.1 * pow(volt , -0.904);
	return distance;
}

int rightSensorReading(){
	//Sharp 3
	float volt = (float)SensorValue[rightIRSensor]/4096*5;
	float distance = 27.534 * pow(volt , -1.207);
	return distance;
}

int topSensorReading(){
	//Sharp 1
	float volt = (float)SensorValue[topIRSensor]/4096*5;
	//TODO: update equation
	float distance = 27.534 * pow(volt , -1.207);
	return distance;
}

int backSensorReading(){
	//Sharp 4
	float volt = (float)SensorValue[topIRSensor]/4096*5;
	float distance = 15.02 * pow(volt , -1.286);
	return distance;
}

/* Controlling motor speeds for left and right wheels
'leftMotorSpeed' and 'rightMotorSpeed' control speed of each motor
'dir' controls the positive and negative signs for motor speeds to
determine direction of motion-
'f': forward
'l': rotate counter-clockwise on the spot
'r': rotate clockwise on the spot
'b': backward */
void moveMotor(float leftMotorSpeed, float rightMotorSpeed, char dir){
	switch (dir){
	case 'f':
		motor[leftMotor] = (int)(leftMotorSpeed*127);
		motor[rightMotor] = (int)(rightMotorSpeed*127);
		break;
	case 'l':
		motor[leftMotor] = - (int)(leftMotorSpeed*127);
		motor[rightMotor] = (int)(rightMotorSpeed*127);
		break;
	case 'r':
		motor[leftMotor] = (int)(leftMotorSpeed*127);
		motor[rightMotor] = - (int)(rightMotorSpeed*127);
		break;
	case 'b':
		motor[leftMotor] = - (int)(leftMotorSpeed*127);
		motor[rightMotor] = - (int)(rightMotorSpeed*127);
		break;
	}
}

float compass(void){
	int num;
	num = SensorValue[compass_MSB]*pow(2,3) + SensorValue[compass_Bit2]*pow(2,2) + SensorValue[compass_Bit3]*2 + SensorValue[compass_LSB];
	switch(num){
	case 7: return 0; //W
		break;
	case 3: return 45; //SW
		break;
	case 11: return 90; //S
		break;
	case 9: return 135; //SE
		break;
	case 13: return 180;  //E
		break;
	case 12: return 225; //NE
		break;
	case 14: return 270; //N
		break;
	case 6: return 315; //NW
		break;
	}

	return -1;
}

//Initialise struct variables
void initialise(){
	//if(starting_pos = 0) odom.X = 40;
	//else odom.X = 80;
	odom.X = 30;
	odom.Y = 15;
	odom.compass_heading = compass(); //To do
	odom.heading = 90.0;
	body.length = 30.0;
	body.width = 30.0;
	body.wheel_radius = 3.5;
}


#endif
