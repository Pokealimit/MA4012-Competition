#ifndef __ENCODER__
#define __ENCODER__

#include "var_updated.c"

/* Print coordinates and heading onto bluetooth module*/
void print_bluetooth(int choice, bool debug){
	// 1 - Print current coordinates	// 2 - Print Battery Status
	char coord[80];
	static int print_bluetooth_delay = 0;									// Timer state to delay printing 
	static int print_bluetooth_mutex = 1;									// Prevent printing while function is already printing
	if(nSysTime - print_bluetooth_delay > 500 && print_bluetooth_mutex){
		switch(choice){
		case 1:
			print_bluetooth_mutex = 0;
			sprintf(coord, "X = %.0f, Y = %.0f, heading = %.0f \n",odom.X,odom.Y,odom.heading);
			bnsSerialSend(UART1, coord);
			if (debug)	writeDebugStreamLine("X = %.0f, Y = %.0f, heading = %.0f",odom.X,odom.Y,odom.heading);
			print_bluetooth_delay = nSysTime;
			print_bluetooth_mutex = 1;
			break;
		case 2:
			print_bluetooth_mutex = 0;
			sprintf(coord, "Battery: %.2f \n",nAvgBatteryLevel);
			bnsSerialSend(UART1, coord);
			if (debug) writeDebugStreamLine("Battery: ",nAvgBatteryLevel);
			print_bluetooth_delay = nSysTime;
			print_bluetooth_mutex = 1;
			break;
		}
	}
}

/* This task counts the number of times that the reflective
sensor detects a change from 0 to 1, signifying that the
black strip is crossed and the wheel has completed half a
revolution. As it is counting, it also updates the time
difference between each count, stored under 'count_period_left'
and 'count_period_right'. */
task counting(){
	int leftRead = 0;		//Variable to identify previously read state
	int rightRead = 0;  //Variable to identify previously read state
	static int leftcounter = 0;		//Number of times left strip is crossed
	static int rightcounter =0;		//Number of times right strip is crossed
	float current_time_left;
	float current_time_right;
	current_time_left = nSysTime;
	current_time_right = nSysTime;

	while(true){
		if(SensorValue[leftWheelSensor]==1 && leftRead==0){
			leftRead=1;
			leftcounter++;
			if(leftcounter>1){ 		//To eliminate error due to starting position of the encoder, first count is ignored
				count_period_left = nSysTime - current_time_left;
				current_time_left = nSysTime;
			}
			else current_time_left = nSysTime;
			//writeDebugStreamLine("Left count = %d, Left count period = %f",leftcounter,count_period_left/1000.0);
		}
		else if(SensorValue[leftWheelSensor]==0 && leftRead==1){
			leftRead=0;
		}
		if(SensorValue[rightWheelSensor]==1 && rightRead==0){
			rightRead=1;
			rightcounter++;
			if(rightcounter>1){		//To eliminate error due to starting position of the encoder, first count is ignored
				count_period_right = nSysTime - current_time_right;
				current_time_right = nSysTime;
			}
			else current_time_right = nSysTime;
			//writeDebugStreamLine("Right count = %d, Right count period = %f",rightcounter,count_period_right/1000.0);
		}
		else if(SensorValue[rightWheelSensor]==0 && rightRead==1){
			rightRead=0;
		}
		if(motor[leftMotor] == 0 || motor[rightMotor] == 0){
			leftcounter = 0;
			rightcounter = 0;
		}
	}
}


task mapping{
	//interval is in milliseconds
	float interval=10;

	while(true){
		/* Kinematics Equations:
		w_left = 1/(time taken for left wheel to complete half a revolution) * PI rad/s
		w_right = 1/(time taken for right wheel to complete half a revolution) * PI rad/s
		v_X = w_left + w_right * R/2 * cos(heading in rad)
		v_Y = w_left + w_right * R/2 * sin(heading in rad)
		w = (w_right - w_left) * R/L
		where R is the radius of wheel and L is the distance between each wheel */
		if (motor[leftMotor] > 0 && count_period_left > 0)
			odom.w_left = 1.0 / (count_period_left/1000.0) * PI;
		else if(motor[leftMotor] < 0 && count_period_left > 0)
			odom.w_left = - 1.0 / (count_period_left/1000.0) * PI;
		else odom.w_left = 0;
		if (motor[rightMotor] > 0 && count_period_right > 0)
			odom.w_right = 1.0 / (count_period_right/1000.0) * PI;
		else if(motor[rightMotor] < 0 && count_period_right > 0)
			odom.w_right = - 1.0 / (count_period_right/1000.0) * PI;
		else odom.w_right = 0;
		odom.v_X = (odom.w_left + odom.w_right) * 3.5/2.0 * cos(odom.heading/180*PI);
		odom.v_Y = (odom.w_left + odom.w_right) * 3.5/2.0 * sin(odom.heading/180*PI);
		odom.w = (odom.w_right - odom.w_left) * 3.5/25.0;

		/* Updating x,y coordinates and heading
		At every interval, the odometry of the robot is updated by
		taking the current X, Y and heading values and adding them to
		the change (v_X,v_Y,w respectively) multiplied by the interval time */
		if (count_period_left > 0 && count_period_right > 0){
			odom.X = odom.X + odom.v_X * interval/1000.0 * 1.3;		//scaling factor of 1.225 from testing
			odom.Y = odom.Y + odom.v_Y * interval/1000.0 * 1.3;		//scaling factor of 1.225 from testing
			odom.heading = odom.heading + 1.6 * odom.w * 180 / PI * interval/1000.0;
		}

		print_bluetooth(1,0);
		writeDebugStreamLine("X = %f, Y = %f, heading = %f",odom.X,odom.Y,odom.heading);

		sleep(interval);
	}
}

#endif
