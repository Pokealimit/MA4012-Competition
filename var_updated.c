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
float top_distanceThreshold = 17.7;

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
float phase;
int round_count;
bool first_launch;
bool next_launch;
bool phase_change;
int boundary_count = 0;	// 2 - change phase during boundary avoidance
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
	//return 1000;
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
	float volt = (float)SensorValue[backIRSensor]/4096*5;
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
	phase = 90;
	round_count = 1;
	first_launch = true;
	next_launch = false;
	phase_change = false;
	boundary_count = 0;
	//writeDebugStreamLine("Initialising...");
}

/* F U N C T I O N S --------*/
int boundary_avoidance();
int ball_return(void);
int move_straight(float distance);
int pan_by_degree(float degree, char dir);
int pan_to_heading_uncallibrated (float heading);
int pan_to_heading (float heading);
int pan_and_search(float degree, char dir);
int obstacle_avoidance();
int boundary_avoidance_back();

int boundary_avoidance(){
	static int timer = nSysTime;
	if (SensorValue(leftLF) == 0 || SensorValue(rightLF) == 0){	//Checks left line follower

		moving_back_t = nSysTime;
		//bnsSerialSend(UART1, "Boundary avoidance ...\n");
		while (nSysTime - moving_back_t < 750 && SensorValue(Power_Switch) == Power_Switch_ON){ //Move back at 0.3 speed for 0.4 seconds
			moveMotor(0.3,0.3,'b',0);
		}

		boundary_count++;

		if (boundary_count == 1 && phase==180){
			phase -= 90;
			//Increase round count when phase changing from 180 to 90 degrees
			round_count++;
			pan_to_heading(phase);
			boundary_count = 0;
			phase_change = true;
		}

		if (boundary_count < 2){
			pan_to_heading(phase);
		}
		else {
			phase -= 90;
			if (phase < 0) phase = 270;
			pan_to_heading(phase);
			boundary_count = 0;
			phase_change = true;
		}
		//writeDebugStreamLine("return 0");
		return 0;
	}
	//writeDebugStreamLine("return 1");
	return 1;
}

int move_straight(float distance){
	writeDebugStreamLine("Move straight start...");
	//bnsSerialSend(UART1, "Move straight start..\n");
	float current_Y = odom.Y;
	float current_X = odom.X;
	if (distance<0)	char dir = 'b';
	else dir = 'f';
	distance = fabs(distance);

	while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<distance){
		moveMotor(1,1,dir,40);
		motor[roller] = 127;
		motor[servo] = -80;
		if(SensorValue[ball_limit_switch] == Ball_Limit_Switch_CONTACT){
			ball_return();
			return 0;
		}
	}
	moveMotor(0,0,'f',0);
	writeDebugStreamLine("Move straight end...");
	//bnsSerialSend(UART1, "Move straight end..\n");
	if (phase_change) return 0;
	else return 1;
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
	//bnsSerialSend(UART1,"Pan by degree start...\n");
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
	while (SensorValue(Power_Switch) == Power_Switch_ON && fabs(odom.heading-current_heading)<degree){ //&& boundary_avoidance()
		moveMotor(speed,speed,dir,0);
		//motor[roller] = 127;
		//motor[servo] = -70;
		//if(SensorValue[ball_limit_switch] == Ball_Limit_Switch_CONTACT){
		//	ball_return();
		//	return 0;
		//}
	}
	moveMotor(0,0,'f',0);
	writeDebugStreamLine("Pan end...");
	//bnsSerialSend(UART1, "Pan by degree end..\n");
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
	while  (array[pointer] != compass() && SensorValue(Power_Switch) == Power_Switch_ON){ //&& boundary_avoidance()
		if(count_test) {
			//char test1[80]; sprintf(test1,"Moving to %d => Bouncing to %d\n",heading,array[pointer]); bnsSerialSend(UART1,test1);
			count_test = 0;
		}
		float pointer_array_diff = compass() - array[pointer];
		//To account for pointer = 7 (pointing to 315 degrees)
		if(pointer_array_diff < 0) pointer_array_diff += 360;

		if (pointer_array_diff >= 0 && pointer_array_diff <=180 ){
			minimum_t = nSysTime;
			while (nSysTime - minimum_t < 200 ){ //&& boundary_avoidance() Move at least for 1 second for angles close to the landmarks
				moveMotor(0.5,0.5,'r',0);
			}
		}
		else {
			minimum_t = nSysTime;
			while (nSysTime - minimum_t < 200 ){ //&& boundary_avoidance() Move at least for 1 second for angles close to the landmarks
				moveMotor(0.5,0.5,'l',0);
			}
		}
	}
	//char test2[80]; sprintf(test2,"Compass reading : %d\n",compass()); bnsSerialSend(UART1,test2);

	while  (heading != compass() && SensorValue(Power_Switch) == Power_Switch_ON ){ //&& boundary_avoidance()
		if(count_test3){
			//char test3[80]; sprintf(test3,"Bouncing to %d\n",heading);bnsSerialSend(UART1,test3);
			count_test3 = 0;
		}
		writeDebugStreamLine("In pan to heading diff heading");
		moveMotor(0.3,0.3,'l',0);
	}

	//char test4[80]; sprintf(test4,"Compass reading : %d\n",compass()); bnsSerialSend(UART1,test4);

	moveMotor(0,0,'f',0);

	if (heading == compass())  return 1;
	return 0;
}

/*Pan to heading combining compass and encoder
Input: 0, 90, 180 and 270 only*/
int pan_to_heading (float heading){
	if (heading == 0){
		pan_to_heading_uncallibrated(45);
		pan_by_degree(13, 'r');
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
	bnsSerialSend(UART1,"Pan and search start...\n");
	float current_heading = odom.heading;
	int enemy_detected = 0;
	while (SensorValue(Power_Switch) == Power_Switch_ON && fabs(odom.heading-current_heading)<degree){ //&&boundary_avoidance()
		moveMotor(0.75,0.75,dir,0);
		motor[roller] = 127;
		motor[servo] = -80;
		if(leftSensorReading() <= distanceThreshold || rightSensorReading() <= distanceThreshold){
			int current_t = nSysTime;
			bnsSerialSend(UART1,"ball/enemy detected\n");
			//TODO: Overpanning time need to be tuned
			while(nSysTime - current_t < 250){
				if(topSensorReading() < top_distanceThreshold){
					enemy_detected = 1;
					break; //break out of inner while
				}
			}
			if (enemy_detected == 1){
				enemy_detected = 0; //the next ball might still be detected with obstacle, therefore, change back for continue panning
				continue; //continue the outer while
			}
			else{
				switch(dir){
				case 'r':
					current_t = nSysTime;
					while(!(leftSensorReading() <= distanceThreshold || rightSensorReading() <= distanceThreshold) && (nSysTime - current_t < 250))
						moveMotor(0.75,0.75,'l',0);
					moveMotor(0,0,'f',0);
					motor[servo] = -80;
					break;
				case 'l':
					current_t = nSysTime;
					while(!(leftSensorReading() <= distanceThreshold || rightSensorReading() <= distanceThreshold) && (nSysTime - current_t < 250))
						moveMotor(0.75,0.75,'r',0);
					moveMotor(0,0,'f',0);
					motor[servo] = -80;
					break;
				}
			}

			//Following if loop will only go in if no enemy is detected
			//TODO: need to add in pan in reverse direction to compesate for the 'overpanning'

			writeDebugStreamLine("Ball detected...");
			bnsSerialSend(UART1, "Ball detected..\n");
			if(move_straight(75)==0) return 0;
			/*
			// allow the robot to move further straight again if ball still detected
			switch(move_straight(50)){
			case 0: return 0;
			break;
			case 1: if(leftSensorReading()<distanceThreshold || rightSensorReading()<distanceThreshold);
			if(move_straight(50)==0) return 0;
			break;
			}*/
		}
		if(SensorValue[ball_limit_switch]== Ball_Limit_Switch_CONTACT){
			ball_return();
			return 0;
		}
	}
	moveMotor(0,0,'f',0);
	writeDebugStreamLine("Pan end...");
	bnsSerialSend(UART1, "Pan and search end..\n");
	return 1;
}

int ball_return(void){
	int line_detected = 0;
	//check if front line follower detected while reversing
	//0: no line detected
	//1: right line detected
	//-1: left line detected

	writeDebugStreamLine("Ball obtained------------------------------------------------");
	// 1. stop motor
	motor[servo] = -30;
	moveMotor(0,0,'f',0);
	ball_caught = 1;

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

	int reverse_t = nSysTime;
	int stop_t;
	bool checked_right = false;
	float current_X;
	float current_Y;

	while (ball_caught==1 && SensorValue(Power_Switch) == Power_Switch_ON){
		if(sensorValue(rightLF)==0){
			line_detected = 1;
		}
		else if(sensorValue(leftLF)==0){
			line_detected = -1;
		}

		if(compass() == 90 && boundary_avoidance_back()) moveMotor(1,1,'b',0);
		else	pan_to_heading(90);
		// if both back limit switch triggered
		if (SensorValue(back_limit_3) == 0 && SensorValue(back_limit_4) == 0 && backSensorReading() > 20){
			//if no obstacle, stop motor and release ball
			writeDebugStreamLine("Both limit switch compressed");
			writeDebugStreamLine("Reached the ball collection point.");
			//bnsSerialSend(UART1, "Reached ball collection point.\n");
			moveMotor(0, 0, 'b', 0);
			motor[servo] = 100;
			delay(500);
			writeDebugStreamLine("Ball released.");
			//bnsSerialSend(UART1, "Ball released.\n");
			ball_caught = 0;
			ball_collected++;
			ball_search += 1;

			// move servo back to ready for ball position
			motor[servo] = -80;
			phase = 90; 		//reset phase
			odom.Y = 15;
			round_count = 1;
			moveMotor(0,0,'f',0);
			next_launch = true;
			boundary_count = 0;
			if(line_detected == -1) {
				moveMotor(1,0,'f',0);
				delay(500);
				moveMotor(1,1,'f',40);
				delay(500);
				pan_to_heading(90);
			}
			else if (line_detected == 1) {
				moveMotor(0,1,'f',0);
				delay(500);
				moveMotor(1,1,'f',40);
				delay(500);
				pan_to_heading(90);
			}
			return 1;
		}

		if (backSensorReading() <= 20){
			stop_t = nSysTime;
			while (nSysTime - stop_t < 2000 && backSensorReading() <= 20){
				moveMotor(0, 0, 'b', 0);
			}
			if (nSysTime - stop_t >= 1999){
				if (checked_right == false){
					current_Y = odom.Y;
					current_X = odom.X;
					while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<20){
						moveMotor(1,1,'f',40);
					}
					pan_to_heading(0);
					while (SensorValue(Power_Switch) == Power_Switch_ON && SensorValue(leftLF) != 0 && SensorValue(rightLF) != 0){
						moveMotor(0.8, 0.8, 'f', 40);
					}
					line_detected = 1;
					checked_right = true;
				}
				else{
					current_Y = odom.Y;
					current_X = odom.X;
					while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<20){
						moveMotor(1,1,'f',40);
					}
					pan_to_heading(180);

					while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<80){
						moveMotor(0.8, 0.8, 'f', 40);
					}
					line_detected = 0;
					checked_right = false;
				}
				moveMotor(0.3,0.3,'b',0);
				delay(500);
				moveMotor(0, 0, 'f', 0);
				pan_to_heading(90);
				//checked_right = true;
			}
			// pan_to_heading(90);
			reverse_t = reverse_t + nSysTime - stop_t;
		}

		//Ball stuck at back of robot
		if (nSysTime - reverse_t > 8000 && backSensorReading() > 20){
			//	current_Y = odom.Y;
			//	current_X = odom.X;
			//	while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<100){
			//		moveMotor(1,1,'f',40);
			//	}
			//	pan_to_heading(90);
			//	reverse_t = nSysTime;
			//}

			if (checked_right == false){
				current_Y = odom.Y;
				current_X = odom.X;
				while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<20){
					moveMotor(1,1,'f',40);
				}
				pan_to_heading(0);
				while (SensorValue(Power_Switch) == Power_Switch_ON && SensorValue(leftLF) != 0 && SensorValue(rightLF) != 0){
					moveMotor(0.8, 0.8, 'f', 40);
				}
				checked_right = true;
				line_detected = 1;
			}
			else{
				current_Y = odom.Y;
				current_X = odom.X;
				while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<20){
					moveMotor(1,1,'f',40);
				}
				pan_to_heading(180);

				while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance() && obstacle_avoidance() && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<80){
					moveMotor(0.8, 0.8, 'f', 40);
				}
				checked_right = false;
				line_detected = 0;
			}
			moveMotor(0.3,0.3,'b',0);
			delay(500);
			moveMotor(0, 0, 'f', 0);
			pan_to_heading(90);
			reverse_t = nSysTime;
		}

	}

	return 0;
}


/*Perform obstacle avoidance
Output: 0 - heading change for obstacle avoidance
1 - obstacle avoidance is not performed, resume previous action
*/
int obstacle_avoidance(){
	int flag = 1;
	if (topSensorReading() <= top_distanceThreshold && flag == 1){
		flag = 0;
		int current_t = nSysTime;
		while(nSysTime - current_t < 1500){
			moveMotor(0,0,'f',0);
		}
	}

	if (topSensorReading() <= top_distanceThreshold && flag == 0){
		phase -= 90;
		if (phase < 0) phase = 270;
		pan_to_heading(phase);
	}
	else flag = 1;		// 1 - continue straight after enemy clear	// 0 - change phase regardless
		return flag;
}

int boundary_avoidance_back(){
	int moving_front_t;

if (( SensorValue(back_limit_1) == 0 || SensorValue(back_limit_2) == 0) && backSensorReading() > 20){
		moving_front_t = nSysTime;
		//bnsSerialSend(UART1, "Boundary avoidance back ...\n");
		while (nSysTime - moving_front_t < 1000 && SensorValue(Power_Switch) == Power_Switch_ON){ //Move forward at 0.3 speed for 0.75 seconds
			moveMotor(0.3,0.3,'f',40);
		}
		pan_to_heading(90);
		//writeDebugStreamLine("return 0");
		return 0;
	}
	//writeDebugStreamLine("return 1");
	return 1;
}


#endif
