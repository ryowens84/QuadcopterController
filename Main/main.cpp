/*
Ultimate IMU Code

Written by Ryan Owens
SparkFun Electronics
10/1/10

See code repository for most updated version
link

license
*/
#include <stdio.h>
#include <string.h>
#include "LPC214x.h"
#include "target.h"
#include <math.h>
//*******************************************************
//						C Libraries
//*******************************************************
extern "C"{
#include "main_msc.h"

#include "serial.h"
#include "rprintf.h"

#include "timer0.h"
#include "timer0ISR.h"
#include "timer1.h"
#include "timer1ISR.h"

#include "pid.h"
}

//*******************************************************
//						C++ Libs
//*******************************************************
#include "main.h"
#include "ADXL345.h"
#include "HMC5843.h"
#include "ITG3200.h"
#include "sensor.h"
#include "I2C.h"

//*******************************************************
//					Core Functions
//*******************************************************
void bootUp(void);
void reset(void);

//*******************************************************
//					Global Variables
//*******************************************************
char sensors_updated=0;
char sensor_string[20]="Test";
long int timeout=0;

double PIDresult = 0.0;
PID myPID;
//I2C speed_controller;

//*******************************************************
//					Main Code
//*******************************************************
int main (void)
{
	//Initialize ARM I/O
	bootUp();			//Init. I/O ports, Comm protocols and interrupts

	timer0Init(1000000);
	timer0Match(0, 100, interruptOnMatch | resetOnMatch);
	
	timer1Init(1000000);
	timer1Match(0, 1000, interruptOnMatch | resetOnMatch);
	
	accelerometer.begin();
	gyro.begin();
	compass.begin();
	
	PIDInit(&myPID);	//Create space for the PID struct
	
	//	Set PID Coefficients (p=1 I=0 D=1.4 WORKS WELL!)
	myPID.Proportion	= 1.0;	
	myPID.Integral = 0.05;
	myPID.Derivative	= 1.7;
	myPID.SetPoint = 0.0;	//	Set PID Setpoint	
	
	VICIntEnable |= INT_TIMER0|INT_TIMER1;
	rprintf("Stop!!!\n\r");
	timeout = millis();
	while(millis() < timeout+1000);
	rprintf("Starting Calibration...\n\r");
	gyro.calibrate();

	while(1)
	{
		if(timer0IntFlag==1)
		{
			VICIntEnClr |= INT_TIMER0;
			timer0IntFlag=0;
			
			accelerometer.update();
			gyro.update();
			compass.update();
			sensors_updated=1;	
			
			if(IOPIN0 & LED)LED_OFF();
			else LED_ON();
			
			VICIntEnable |= INT_TIMER0;
		}
		
		if(sensors_updated)
		{
			sensors_updated=0;
		
			filter.last_time=filter.this_time;
			filter.this_time=millis();	//Get the current number of milliseconds
			//Calculate Interval Time in milliseconds
			filter.interval=filter.this_time-filter.last_time;
			
			//Populate the RwAcc Array
			filter.fillAccelValues(accelerometer.getX(), accelerometer.getZ());		
			
			//Normalize the Accelerometers gravity vector
			filter.normalizeVector(filter.RwAcc);
			
			if(filter.first_run)
			{
				for(int w=0; w<2; w++)filter.RwGyro[w] = filter.RwAcc[w];
				filter.first_run-=1;			
			}
			else
			{
				//If the previous estimated values is too small, don't calc. a new one as the error will be large.
				if(filter.RwEst[1] < 0.1)
				{
					for(int w=0; w<2; w++)filter.RwGyro[w]=filter.RwEst[w];
				}
				//Else, find the 'gyro angle' and calculate the weighted average to find attitude of device.
				else
				{
					
					filter.this_rate=gyro.getX();	//Get the current deg/sec from gyroscope.
					filter.this_angle=filter.this_rate*(filter.interval/1000);	//degree/sec * seconds == degrees
					
					filter.Axz = atan2(filter.RwEst[0], filter.RwEst[1])*180/PI;	//Get previous angle in degrees
					filter.Axz += filter.this_angle;	//Add the current angle to the previous one to get current position.
				}
				
				//Find out if RzGyro is positive or negative y checking quadrant of the Axz angle
				if(cos(filter.Axz * (PI/180)) > 0)filter.signRzGyro=1;
				else filter.signRzGyro=-1;

				//Use Axz to find RxGyro and RzGyro
				filter.RwGyro[0] = sin(filter.Axz * (PI/180));
				filter.RwGyro[0] /= sqrt(1);
				filter.RwGyro[1] = filter.signRzGyro * sqrt(1-pow(filter.RwGyro[0],2));	
			}
			//Now we have the gravity force vector from both accelerometer and gyro. Combine them using weighted average
			//to find Rw
			for(int w=0; w<2; w++)
			{
				filter.RwEst[w] = (filter.RwAcc[w] + filter.RwGyro[w] * filter.gyro_weight)/(1+filter.gyro_weight);
			}	
			filter.normalizeVector(filter.RwEst);
			
			filter.AccTheta=atan2(filter.RwAcc[0], filter.RwAcc[1])*180/PI;
			filter.EstTheta=atan2(filter.RwEst[0], filter.RwEst[1])*180/PI;	
		
			PIDresult=PIDCalc(&myPID, filter.EstTheta);
		
			//sprintf(&sensor_string[0], "%1.3f, %1.3f, %1.3f\n\r", filter.interval/1000, filter.AccTheta, filter.EstTheta);
			//rprintf("%s", sensor_string);
			//sprintf(&sensor_string[0], "%1.3f, %1.3f, %1.3f\n\r", filter.interval/1000, filter.RwAcc[0], filter.RwEst[0]);
			//rprintf("%s", sensor_string);			
			sprintf(&sensor_string[0], "%1.3f\n\r", PIDresult);
			rprintf("%s", sensor_string);
			
		}
		
		//If a USB Cable gets plugged in, stop everything!
		if(IOPIN0 & (1<<23))
		{
			VICIntEnClr = INT_TIMER0 | INT_TIMER1;	//Stop all running interrupts			
			main_msc();								//Open the mass storage device
			reset();								//Reset to check for new FW
		}		
	}
	
	
    return 0;
}

//Usage: bootUp();
//Inputs: None
//This function initializes the serial port, the SD card, the I/O pins and the interrupts
void bootUp(void)
{
	//Initialize UART for RPRINTF
    rprintf_devopen(putc_serial0); //Init rprintf
	init_serial0(9600);		
	
	//Initialize I/O Ports and Peripherals
	IODIR0 |= (LED| XBEE_EN);
	
    //Setup the Interrupts
	//Enable Interrupts
	VPBDIV=1;										// Set PCLK equal to the System Clock	
	VICIntSelect = ~(INT_TIMER0|INT_TIMER1);
	VICVectCntl0 = 0x20 | 4;						//Timer 0 Interrupt
	VICVectAddr0 = (unsigned int)ISR_Timer0;
	VICVectCntl1 = 0x20 | 5;						//Timer 1 Interrupt
	VICVectAddr1 = (unsigned int)ISR_Timer1;	
	
}

//Usage: reset();
//Inputs: None
//Description: Resets the LPC2148
void reset(void)
{
    // Intentionally fault Watchdog to trigger a reset condition
    WDMOD |= 3;
    WDFEED = 0xAA;
    WDFEED = 0x55;
    WDFEED = 0xAA;
    WDFEED = 0x00;
}

	/*
	speed_controller.configure();
	while(1)
	{
		for(int i=0; i<10; i++)
		{
			sensor_string[0]=0;
			sensor_string[1]=i*10;
			speed_controller.send(0xB6, sensor_string, WRITE, 2);
			delay_ms(10);
		}
		for(int i=0; i<10; i++)
		{
			sensor_string[0]=1;
			sensor_string[1]=i*10;
			speed_controller.send(0xB6, sensor_string, WRITE, 2);
			delay_ms(10);
		}
		for(int i=0; i<10; i++)
		{
			sensor_string[0]=2;
			sensor_string[1]=i*10;
			speed_controller.send(0xB6, sensor_string, WRITE, 2);
			delay_ms(10);
		}
		for(int i=0; i<10; i++)
		{
			sensor_string[0]=3;
			sensor_string[1]=i*10;
			speed_controller.send(0xB6, sensor_string, WRITE, 2);
			delay_ms(10);
		}		
	}
	*/
