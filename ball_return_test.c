#pragma config(UART_Usage, UART1, uartUserControl, baudRate9600, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in1,    leftIRSensor,   sensorNone)
#pragma config(Sensor, in2,    rightIRSensor,  sensorAnalog)
#pragma config(Sensor, in3,    topIRSensor,    sensorAnalog)
#pragma config(Sensor, in4,    backIRSensor,   sensorAnalog)
#pragma config(Sensor, in5,    back_limit_1,    sensorAnalog)
#pragma config(Sensor, in6,    back_limit_2,   sensorAnalog)
#pragma config(Sensor, dgtl1,  Power_Switch,   sensorDigitalIn)
#pragma config(Sensor, dgtl2,  leftWheelSensor, sensorDigitalIn)
#pragma config(Sensor, dgtl3,  rightWheelSensor, sensorDigitalIn)
#pragma config(Sensor, dgtl4,  ball_limit_switch, sensorDigitalIn)
#pragma config(Sensor, dgtl6,  leftLF,         sensorDigitalIn)
#pragma config(Sensor, dgtl7,  rightLF,        sensorDigitalIn)
#pragma config(Sensor, dgtl8,  compass_LSB,    sensorDigitalIn)
#pragma config(Sensor, dgtl9,  compass_Bit3,   sensorDigitalIn)
#pragma config(Sensor, dgtl10, compass_Bit2,   sensorDigitalIn)
#pragma config(Sensor, dgtl11, compass_MSB,    sensorDigitalIn)
#pragma config(Motor,  port2,           leftMotor,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           rightMotor,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           roller,        tmotorServoContinuousRotation, openLoop)
#pragma config(Motor,  port5,           servo,         tmotorServoStandard, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "BNSlib_HC05.h"
#include "var_updated.c"
#include "encoder_updated.c"

int move_straight(float distance);
int pan_by_degree(float degree, char dir);
int pan_to_heading(float heading);
int ball_return();

task main(){
	//Initialise variables and start multi-tasking for encoder tasks
	initialise();
	setBaudRate(UART1, baudRate9600);
	startTask(counting);
	startTask(mapping);

	while(true){
		while (SensorValue(Power_Switch) == Power_Switch_ON && ball_caught == 0){
			move_straight(20);
		}
		//If power is off, stop all motors and reinitalise variables
		if(SensorValue(Power_Switch) == Power_Switch_OFF){
			moveMotor(0,0,'f',0);
			motor(roller)=0;
			motor(servo)=-85;
			initialise();
		}
	}
}

int move_straight(float distance){
	writeDebugStreamLine("Move straight start...");
	bnsSerialSend(UART1, "Move straight start..\n");
	float current_Y = odom.Y;
	float current_X = odom.X;
	while (SensorValue(Power_Switch) == Power_Switch_ON && ball_caught == 0 && sqrt(pow(odom.Y-current_Y,2)+pow(odom.X-current_X,2))<distance){
		moveMotor(1,1,'f',40);
		motor[roller] = 127;
		motor[servo] = -100;
		if(SensorValue[ball_limit_switch] == Ball_Limit_Switch_CONTACT){
			ball_return();
			return 0;
		}
	}
	writeDebugStreamLine("Move straight end...");
	bnsSerialSend(UART1, "Move straight end..\n");
	return 1;
}

/* Pan in 'dir' direction by 'degree' degrees
'dir' = 'l' : counter-clockwise
'dir' = 'r' : clockwise */
int pan_by_degree(float degree, char dir){
	writeDebugStreamLine("Pan start...");
	bnsSerialSend(UART1,"Pan start...\n");
	float current_heading = odom.heading;
	while (SensorValue(Power_Switch) == Power_Switch_ON && ball_caught == 0 && fabs(odom.heading-current_heading)<degree){
		moveMotor(0.5,0.5,dir,0);
		motor[roller] = 127;
		motor[servo] = -100;
	}
	writeDebugStreamLine("Pan end...");
	bnsSerialSend(UART1, "Pan end..\n");
	return 1;
}


/* Pan to specific heading using compass reading ONLY
Only pan to the 'landmark'-
Input: Angle(in degree) w.r.t x-axis
Return: 0 : Action incomplete
1 : Action complete
*/
int pan_to_heading(float heading){
	float array[8] = {0,45,90,135,180,225,270,315};
	int pointer = 0;
	if (heading == compass()){
		moveMotor(0,0,'f',0);
		sleep(100);
		writeDebugStreamLine("In pan to heading same heading");
		pointer = (heading / 45) - 1;
		if (pointer < 0) pointer = 7;
		while  (array[pointer] != compass() && SensorValue(Power_Switch) == Power_Switch_ON){
			if (compass() - array[pointer] > 0 && compass()-array[pointer] <=180 )
				moveMotor(0.5,0.5,'r',0);
			else moveMotor(0.5,0.5,'l',0);
		}
	}


		moveMotor(0,0,'f',0);
		sleep(100);


	while  (heading != compass() && SensorValue(Power_Switch) == Power_Switch_ON){
		writeDebugStreamLine("In pan to heading diff heading");
		if (compass() - heading > 0 && compass()-heading <=180 ){
			moveMotor(0.5,0.5,'r',0);
		}
		else
			moveMotor(0.5,0.5,'l',0);
	}

	moveMotor(0,0,'f',0);
	sleep(100);

	if (heading == compass())  return 1;
	return 0;
}

/* When ball is caught, return to collection point and drop off ball
1) Stop motor
2) Reverse roller for 1.5s
3) Stop roller
4) Move back to starting point
5) Drop off ball */
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
	while (nSysTime - ball_caught_t < 1500 && ball_caught==1){
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

	while (pan_to_heading(90)!=1){
		pan_to_heading(90);
	}

	while (ball_caught==1 && SensorValue(Power_Switch) == Power_Switch_ON){
		// if both back limit switch triggered
		if (SensorValue[back_limit_1] == 0 && SensorValue[back_limit_2] == 0){
			// check if there is an obstacle
			//if (backSensorReading() > 20){
			//	moveMotor(0,0,'b');
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
			return 0;
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
	return 1;
}
