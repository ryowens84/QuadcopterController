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
#include <stdlib.h>
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

#include "uart0.h"
#include "uart0ISR.h"

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
#include "Compass.h"
#include "DCM.h"
#include "Vector.h"
#include "matrix.h"

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
int active=0;

double PIDresult = 0.0;
int16_t power=0;
PID myPID;

#define NUM_SAMPLES	16
int sampleNumber=0;

double xAccelVal=0;
double yAccelVal=0;
double zAccelVal=0;
double xGyroVal=0;
double yGyroVal=0;
double zGyroVal=0;

double newP=0;
double newI=0;
double newD=0;

int ledStatus=0;


//Variables needed for implementation of DCM Algorithm
int SENSOR_SIGN[9] = {-1,1,-1,1,1,1,-1,-1,-1};  //Correct directions x,y,z - gyros, accels, magnetormeter
float G_Dt=0.022;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;

int AN[6]; //array that stores the raw sensor data (gyro X, Y, Z, accel X, Y, Z)
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int ACC[3]; //array that store the accelerometers data (may be able to get rid of this.

int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;

//*******************************************************
//					Main Code
//*******************************************************
int main (void)
{
	//Initialize ARM I/O
	bootUp();			//Init. I/O ports, Comm protocols and interrupts

	timer0Init(1000000);
	//Set timer interrupts for 800 Hz
	timer0Match(0, 800, interruptOnMatch | resetOnMatch);
	
	//timer 1 is used for the millis() function and shouldn't be changed!
	timer1Init(1000000);
	timer1Match(0, 1000, interruptOnMatch | resetOnMatch);
	
	//Set the UART interrupt to trigger when a single character is received
	uart0RxInt(RX0_TRIG_LEV_0);	
	
	accelerometer.begin();
	gyro.begin();
	compass.begin();
	
	PIDInit(&myPID);	//Create space for the PID struct
	
	//	Set PID Coefficients (p=1 I=0 D=1.4 WORKS WELL!)
	myPID.Proportion	= -1.2;	
	myPID.Integral = 0.3;
	myPID.Derivative	= 0.0;
	myPID.SetPoint = 0.0;	//	Set PID Setpoint	
	
	//Enable interrupts for timers and uart
	VICIntEnable |= INT_TIMER0|INT_TIMER1| INT_UART0;
	
	LED_ON();
	timeout=millis();
	while(millis() < timeout+500);
	LED_OFF();
	//rprintf("Stop!!!\n\r");
	timeout = millis();
	while(millis() < timeout+500);
	//rprintf("Starting Calibration...\n\r");
	gyro.calibrate();
	accelerometer.calibrate();
	//Fill the offset array
	AN_OFFSET[0] = gyro.getXOffset();
	AN_OFFSET[1] = gyro.getYOffset();
	AN_OFFSET[2] = gyro.getZOffset();
	AN_OFFSET[3] = accelerometer.getXOffset();
	AN_OFFSET[4] = accelerometer.getYOffset();
	AN_OFFSET[5] = accelerometer.getZOffset();
	//Compensate the Z-Axis reading for gravity (to set to 0 when facing up)
	AN_OFFSET[5] -= GRAVITY*SENSOR_SIGN[5];	

	COMM_OFF();	//Start communication with the motor controller.
	timeout = millis();
	timer = millis();
	while(1)
	{
		//Turn off communication if we've timed out!
		if((millis() > timeout+800)||active==0)COMM_OFF();
		else COMM_ON();
		if(timer0IntFlag==1)
		{
			VICIntEnClr |= INT_TIMER0;
			timer0IntFlag=0;
			
			accelerometer.update();
			gyro.update();
			compass.update();
			
			if(sampleNumber == 0){
				xAccelVal=0;
				yAccelVal=0;
				zAccelVal=0;
				xGyroVal=0;
				yGyroVal=0;
				zGyroVal=0;
			}
			xAccelVal+=accelerometer.getX();
			yAccelVal+=accelerometer.getY();
			zAccelVal+=accelerometer.getZ();
			xGyroVal+=gyro.getX();
			yGyroVal+=gyro.getY();
			zGyroVal+=gyro.getZ();
			sampleNumber+=1;
			if(sampleNumber == NUM_SAMPLES){
				sampleNumber=0;
				sensors_updated=1;
				xAccelVal/=NUM_SAMPLES;
				yAccelVal/=NUM_SAMPLES;
				zAccelVal/=NUM_SAMPLES;
				xGyroVal/=NUM_SAMPLES;
				yGyroVal/=NUM_SAMPLES;
				zGyroVal/=NUM_SAMPLES;				
			}
			
			VICIntEnable |= INT_TIMER0;
		}
		
		//Check for incoming message from XBee
		if(uart0MessageComplete)
		{
			VICIntEnClr |= INT_UART0;
			
			timeout = millis();	
			if(ledStatus==1){
				LED_OFF();
				ledStatus=0;
			}
			else{
				LED_ON();
				ledStatus=1;
			}			
			
			//A 0 in the first index of the message means we can power up.
			if(uart0Message[0]==0){
				active=1;
			}
			//If there's anything else, stop communicating.
			else{
				active=0;
			}	
			
			//Make sure we get enough characters
			if(strlen(uart0Message) >= 4){		
				
				newP = (double)uart0Message[1]/100.0;	
				newI = (double)uart0Message[2]/100.0;
				newD = (double)uart0Message[3]/100.0;	

				if((newP != myPID.Proportion)||(newI != myPID.Integral)||(newD!=myPID.Derivative)){					
					myPID.Proportion = newP*-1;	
					myPID.Integral = newI;
					//myPID.Derivative = newD;
				}
			}
			uart0MessageComplete=0;
			//Enable Interrupts again.
			VICIntEnable |= INT_UART0;
		}		
		
		//If the sensors have been updated, it's time to run the DCM algorithm and send new vals to the motors
		if(sensors_updated)
		{
			//Update the timer
			//timer_old = timer;
			//timer = millis();
			//if(timer>timer_old)G_Dt = (timer-timer_old);///1000.0;
			//else G_Dt = 0;
			
			//Update the sensor values
			AN[0] = (int)gyro.getX();
			AN[1] = (int)gyro.getY();
			AN[2] = (int)gyro.getZ();
			AN[3] = (int)accelerometer.getX();
			AN[4] = (int)accelerometer.getY();
			AN[5] = (int)accelerometer.getZ();		
			accel_x = SENSOR_SIGN[3]*(AN[3]-AN_OFFSET[3]);
			accel_y = SENSOR_SIGN[4]*(AN[4]-AN_OFFSET[4]);
			accel_z = SENSOR_SIGN[5]*(AN[5]-AN_OFFSET[5]);

			//Find the compass heading
			magnetom_x = (int)compass.getX();
			magnetom_y = (int)compass.getY();
			magnetom_z = (int)compass.getZ();
			Compass_Heading();
			
			//Run the DCM Calculations on the new sensor readings
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();
		
			//PIDresult=PIDCalc(&myPID, filter.AccTheta);
			PIDresult=PIDCalc(&myPID, ToDeg(pitch));			
			power = (int16_t)PIDresult;
			power/=2;
			
			//Limit the output power.
			if(abs(power) >=8)
			{
				if(power > 0)power = 8;
				else power = -8;
			}
			
			//send the speed values to the speed controller
			sprintf(&sensor_string[0], "%c%c%c%c%c%c", 0x66, (char)10+power, (char)10-power, 0x01, 0x01, 0x67);
			
			//Send the string to the controller
			rprintf_devopen(putc_serial1); //Init rprintf
			rprintf(sensor_string);
			
			//Send the system information to the XBee
			sprintf(&sensor_string[0], "%c%c%c%c%c%2d%c", 0x66, (char)10+power, (char)10-power, 0x01, 0x01,
				(int)ToDeg(pitch), 0x67);

			rprintf_devopen(putc_serial0); //Init rprintf
			rprintf(sensor_string); 
			
			//Toggle the LED
			//if(LED & IOPIN0 == LED)LED_OFF();
			//else LED_ON();
				
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
    rprintf_devopen(putc_serial1); //Init rprintf
	init_serial1(57600);
	init_serial0(9600);
	
	
	PINSEL0 &= ~(3<<((12+1)*2));	//Set the Comm pin as GPIO in the Pin Select Register
	IODIR0 |= COMM;						//Sets the COMM pin as an output
	COMM_OFF();							//Start the program with communication off.

	//Initialize I/O Ports and Peripherals
	IODIR0 |= (LED| XBEE_EN);
	
	
	//Turn on the XBee module
	XBEEon();
	
    //Setup the Interrupts
	//Enable Interrupts
	VPBDIV=1;										// Set PCLK equal to the System Clock	
	VICIntSelect = ~(INT_TIMER0|INT_TIMER1|INT_UART0);
	VICVectCntl0 = 0x20 | 4;						//Timer 0 Interrupt
	VICVectAddr0 = (unsigned int)ISR_Timer0;
	VICVectCntl1 = 0x20 | 5;						//Timer 1 Interrupt
	VICVectAddr1 = (unsigned int)ISR_Timer1;	
	VICVectCntl2 = (0x20 | 6);
	VICVectAddr2 = (unsigned int)ISR_UART0;			//UART 1 Interrupt	
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
