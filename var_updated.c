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
float distanceThreshold = 25.0;

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

/* --n-----------------------
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
int ball_found; //if ball is detected by IR sensor
int ball_collected;	//number of balls in collection area
int ball_caught; //if ball is detected by ball limit switch
int roll_forward;
int boundary;   //0: bottom, 1: left, 2: top, 3: right
int ball_search; 		//0: move to 2/3, 1: subsequent search
int counter;
int phase;
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

/* Controlling motor speeds for left and right wheels
'leftMotorSpeed' and 'rightMotorSpeed' control speed of each motor
'dir' controls the positive and negative signs for motor speeds to
determine direction of motion-
'f': forward
'l': rotate counter-clockwise on the spot
'r': rotate clockwise on the spot
'b': backward */
void moveMotor(float leftMotorSpeed, float rightMotorSpeed, char dir, int time){
	switch (dir){
	case 'f':
		motor[rightMotor] = (int)(rightMotorSpeed*127);
		sleep(time);
		motor[leftMotor] = (int)(leftMotorSpeed*127);
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
	odom.Y = 30;
	odom.compass_heading = compass(); //To do
	odom.heading = 90.0;
	body.length = 30.0;
	body.width = 30.0;
	body.wheel_radius = 3.5;
	ball_found = 0;
	ball_collected = 0;
	ball_caught = 0;
	roll_forward = 1;
	boundary = 0;
	ball_search = 1;
	counter = 0;
	phase = 1;
	//writeDebugStreamLine("Initialising...");
}

/* F U N C T I O N S --------*/
int boundary_avoidance();
int ball_return(void);
int move_straight(float distance);
int pan_by_degree(float degree, char dir);
int pan_to_heading (float heading);
int pan_and_search(float degree, char dir);

int boundary_avoidance(){
	int prev_heading = phase; //Store heading when hit boundary

	if (SensorValue(leftLF) == 0){	//Checks left line follower
		moving_back_t = nSysTime;
		bnsSerialSend(UART1, "Boundary avoidance ...\n");
		while (nSysTime - moving_back_t < 400 && SensorValue(Power_Switch) == Power_Switch_ON){ //Move back at 0.3 speed for 0.4 seconds
			moveMotor(0.3,0.3,'b',0);
		}

		if (phase == 1) pan_to_heading(90); //Face back to South
		else if (phase ==2) pan_to_heading(0); //Face back to West
		else if (phase == 3) pan_to_heading(270);//Face back to North
		else if ( phase==4 ) pan_to_heading(180);//Face back to East

		//writeDebugStreamLine("return 0");
		return 0;
	}

	if (SensorValue(rightLF) == 0){	//Checks right line follower
		moving_back_t = nSysTime;
		bnsSerialSend(UART1, "Boundary avoidance ...\n");
		while (nSysTime - moving_back_t < 400 && SensorValue(Power_Switch) == Power_Switch_ON){ //Move back at 0.3 speed for 0.4 seconds
			moveMotor(0.3,0.3,'b',0);
		}

		if (phase == 1) pan_to_heading(90); //Face back to South
		else if (phase ==2) pan_to_heading(0); //Face back to West
		else if (phase == 3) pan_to_heading(270);//Face back to North
		else if ( phase==4 ) pan_to_heading(180);//Face back to East

		//writeDebugStreamLine("return 0");
		return 0;
	}

	//writeDebugStreamLine("return 1");
	return 1;
}

int move_straight(float distance){
	writeDebugStreamLine("Move straight start...");
	bnsSerialSend(UART1, "Move straight start..\n");
	float current_Y = odom.Y;
	float current_X = odom.X;
	if (distance<0)	char dir = 'b';
	else dir = 'f';
	distance = fabs(distance);

	while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<distance){
		moveMotor(1,1,dir,40);
		motor[roller] = 127;
		motor[servo] = -100;
		if(SensorValue[ball_limit_switch] == Ball_Limit_Switch_CONTACT){
			ball_return();
			return 0;
		}
	}
	moveMotor(0,0,'f',0);
	writeDebugStreamLine("Move straight end...");
	bnsSerialSend(UART1, "Move straight end..\n");
	return 1;
}

/*
Generic function to pan the robot by number of degrees specified by degree, in the direction specified by dir
- ?l?: left/counter-clockwise
- ?r?: right/clockwise
When the function starts, its current heading is updated from the robot?s odometry (current_heading = odom.heading)
A while() loop repeatedly commands the robot to turn on the spot in the specified direction until the difference between its constantly-updated heading (odom.heading) and its initial heading (current_heading) is equal/more (>=) than degree
Within the loop, these states are polled constantly:
- Power_Switch_ON/OFF
- Left/Right Line Followers (Boundary Avoidance)
At different degrees,
*/
int pan_by_degree(float degree, char dir){
	writeDebugStreamLine("Pan start...");
	bnsSerialSend(UART1,"Pan start...\n");
	float current_heading = odom.heading;
	float speed;
	switch (degree){
	case 180: speed = 1;
		break;
	case 90: speed = 0.5;
		break;
	default: speed = 0.3;
		break;
	}
	while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && fabs(odom.heading-current_heading)<degree){
		moveMotor(speed,speed,dir,0);
		motor[roller] = 127;
		motor[servo] = -100;
		if(SensorValue[ball_limit_switch] == Ball_Limit_Switch_CONTACT){
			ball_return();
			return 0;
		}
	}
	moveMotor(0,0,'f',0);
	writeDebugStreamLine("Pan end...");
	bnsSerialSend(UART1, "Pan end..\n");
	return 1;
}

//Pan using compass reading only
int pan_to_heading_uncallibrated(float heading){
	float array[8] = {0,45,90,135,180,225,270,315};
	int pointer = 0;
	int count_test;
	int count_test3;
	int minimum_t;

	pointer = (heading / 45) - 1;
	if (pointer < 0)	pointer = 7;
	while  (array[pointer] != compass() && SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance()){
		if(count_test) {
			char test1[80]; sprintf(test1,"Moving to %d => Bouncing to %d\n",heading,array[pointer]); bnsSerialSend(UART1,test1);
			count_test = 0;
		}
		float pointer_array_diff = compass() - array[pointer];
		//To account for pointer = 7 (pointing to 315 degrees)
		if(pointer_array_diff < 0) pointer_array_diff += 360;

		if (pointer_array_diff >= 0 && pointer_array_diff <=180 ){
			minimum_t = nSysTime;
			while (nSysTime - minimum_t < 200 && boundary_avoidance()){ //Move at least for 1 second for angles close to the landmarks
				moveMotor(0.5,0.5,'r',0);
			}
		}
		else {
			minimum_t = nSysTime;
			while (nSysTime - minimum_t < 200 && boundary_avoidance()){ //Move at least for 1 second for angles close to the landmarks
				moveMotor(0.5,0.5,'l',0);
			}
		}
	}
	char test2[80]; sprintf(test2,"Compass reading : %d\n",compass()); bnsSerialSend(UART1,test2);

	while  (heading != compass() && SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance()){
		if(count_test3){
			char test3[80]; sprintf(test3,"Bouncing to %d\n",heading);bnsSerialSend(UART1,test3);
			count_test3 = 0;
		}
		writeDebugStreamLine("In pan to heading diff heading");
		moveMotor(0.3,0.3,'l',0);
	}

	char test4[80]; sprintf(test4,"Compass reading : %d\n",compass()); bnsSerialSend(UART1,test4);

	moveMotor(0,0,'f',0);

	if (heading == compass())  return 1;
	return 0;
}

/*Pan to heading combining compass and encoder
Input: 0, 90, 180 and 270 only*/
int pan_to_heading (float heading){
	if (heading == 0){
		pan_to_heading_uncallibrated(45);
		pan_by_degree(25, 'r');
	}
	else if (heading == 90){
		pan_to_heading_uncallibrated(90);
	}
	else if (heading == 180){
		pan_to_heading_uncallibrated(135);
		pan_by_degree(40, 'l');
	}
	else if (heading == 270){
		pan_to_heading_uncallibrated(315);
		pan_by_degree(10, 'r');
	}
	return 1;
}

int pan_and_search(float degree, char dir){
	writeDebugStreamLine("Pan start...");
	bnsSerialSend(UART1,"Pan start...\n");
	float current_heading = odom.heading;
	while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && fabs(odom.heading-current_heading)<degree){
		moveMotor(0.75,0.75,dir,0);
		motor[roller] = 127;
		motor[servo] = -100;
		if(leftSensorReading()<distanceThreshold || rightSensorReading()<distanceThreshold){
			writeDebugStreamLine("Ball detected...");
			bnsSerialSend(UART1, "Ball detected..\n");
			if (move_straight(50)) move_straight(-50);
		}
		if(SensorValue[ball_limit_switch]== Ball_Limit_Switch_CONTACT){
			ball_return();
			return 0;
		}
	}
	moveMotor(0,0,'f',0);
	writeDebugStreamLine("Pan end...");
	bnsSerialSend(UART1, "Pan end..\n");
	return 1;
}

int ball_return(void){
	//float current_heading;
	writeDebugStreamLine("Ball obtained------------------------------------------------");
	// 1. stop motor
	moveMotor(0,0,'f',0);
	ball_caught = 1;
	motor[servo] = -45;

	// 2. reverse roller for 1.5s to ensure only 1 ball is caught
	writeDebugStreamLine("2. Reversing roller");
	ball_caught_t = nSysTime;
	while (nSysTime - ball_caught_t < 1500 && ball_caught==1 && boundary_avoidance()){
		motor[roller] = -127;
		roll_forward = 0;
	}
	// 3. stop roller
	writeDebugStreamLine("3. Stop roller");
	motor[roller] = 0;

	// 4. Move back to starting point
	// Turn to face South
	writeDebugStreamLine("4. Face south");
	writeDebugStreamLine("Heading: %.2f", odom.heading);

	//while (pan_to_heading(90)!=1){
	pan_to_heading(90);
	//}

	while (ball_caught==1 && SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance()){
		// if both back limit switch triggered
		if (SensorValue[back_limit_1] == 0 && SensorValue[back_limit_2] == 0){
			//stop motor
			moveMotor(0, 0, 'b', 0);
			// check if there is an obstacle
			//if (backSensorReading() > 20){
			//	moveMotor(0,0,'b',0);
			//}
			// if no obstacle, release ball
			//else{
			writeDebugStreamLine("Both limit switch compressed");
			writeDebugStreamLine("Reached the ball collection point.");
			bnsSerialSend(UART1, "Reached ball collection point.\n");
			motor[servo] = 100;
			delay(500);
			writeDebugStreamLine("Ball released.");
			bnsSerialSend(UART1, "Ball released.\n");
			ball_caught = 0;
			ball_collected++;
			ball_search += 1;

			// move servo back to ready for ball position
			motor[servo] = -100;
			phase = 1; 		//reset phase

			odom.y = 15;

			////move back to (30, 30)
			//float current_x = odom.x;
			//float x_dist = odom.x - 30;
			///*
			//1. move forward slightly
			//2. turn left 90 deg
			//3. move straight x_dist
			//4. turn right 90 deg
			//*/
			//move_straight(15);
			//pan_by_degree(90, 'l');
			//move_straight(x_dist);
			//pan_by_degree(90, 'r');
			//// need to reverse again?
			//// moveMotor(0.5,0.5,'b');
			//// sleep(500);
			moveMotor(0,0,'f',0);

			return 1;
			//}
		}
		// if both back limit switch not compressed
		//else if (SensorValue[back_limit_1] != 0 && SensorValue[back_limit_2] != 0){
		//	// and if back IR sensor sensing > 20cm
		//	if (backSensorReading() > 20){
		//		// if there is no obstacles at the back, reverse
		//		moveMotor(1,1,'b');
		//	}
		//	else{
		//		// if got ostacle, stop
		//		moveMotor(0,0,'b');
		//	}
		//}

		// if only one limit switch triggered
		else {
			// stop
			//writeDebugStreamLine("Reverse");
			moveMotor(1,1,'b',0);
		}
	}
	return 0;
}


#endif
