#pragma config(UART_Usage, UART1, uartUserControl, baudRate9600, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartNotUsed, baudRate4800, IOPins, None, None)
#pragma config(Sensor, in3,    leftIRSensor,   sensorAnalog)
#pragma config(Sensor, in2,    rightIRSensor,  sensorAnalog)
#pragma config(Sensor, in1,    topIRSensor,    sensorAnalog)
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


task main(){
	//Initialise variables and start multi-tasking for encoder tasks
	initialise();
	setBaudRate(UART1, baudRate9600);
	startTask(counting);
	startTask(mapping);

	while(true){
		writeDebugStreamLine("Battery: %d",nAvgBatteryLevel);

		while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance()){
			//main code here:

			if (phase == 0)		bnsSerialSend(UART1, "Phase: 0..\n");
			if (phase == 90)	bnsSerialSend(UART1, "Phase: 90..\n");
			if (phase == 180)	bnsSerialSend(UART1, "Phase: 180..\n");
			if (phase == 270)	bnsSerialSend(UART1, "Phase: 270..\n");

			//bnsSerialSend(UART1, "Moving straight..\n");
			if (move_straight(60)==0 || SensorValue(Power_Switch) == Power_Switch_OFF) break;
			sleep(500);
			//bnsSerialSend(UART1, "Panning..\n");
			if (pan_and_search(180, 'r') == 0 || SensorValue(Power_Switch) == Power_Switch_OFF) break;
			sleep(500);
			if (pan_and_search(180, 'r') == 0 || SensorValue(Power_Switch) == Power_Switch_OFF) break;
			sleep(500);

			pan_to_heading(phase);		// Reorientate back to the phase as pan_by_degree is not extremely accurate

		}
		//If power is off, stop all motors and reinitalise variables
		if(SensorValue(Power_Switch) == Power_Switch_OFF){
			moveMotor(0,0,'f',0);
			motor(roller)=0;
			motor(servo)=-85;
			initialise();
			//print_bluetooth(2,0);
		}
	}
}

//task main(){
//	//Initialise variables and start multi-tasking for encoder tasks
//	initialise();
//	setBaudRate(UART1, baudRate9600);
//	startTask(counting);
//	startTask(mapping);

//	while(true){
//		while (SensorValue(Power_Switch) == Power_Switch_ON && boundary_avoidance()){
//			 move_straight(30);
//			 pan_to_heading(90);
//			 sleep(500);
//		}
//	}
//}