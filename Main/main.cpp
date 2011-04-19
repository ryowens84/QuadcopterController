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
float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
int AN[6]; //array that store the 3 ADC filtered data (gyros)
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int ACC[3];          //array that store the accelerometers data

int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;


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
	rprintf("Stop!!!\n\r");
	timeout = millis();
	while(millis() < timeout+1000);
	rprintf("Starting Calibration...\n\r");
	gyro.calibrate();

	COMM_OFF();	//Start communication with the motor controller.
	timeout = millis();
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
					if(ledStatus==1){
						LED_OFF();
						ledStatus=0;
					}
					else{
						LED_ON();
						ledStatus=1;
					}	
				
					myPID.Proportion = newP*-1;	
					myPID.Integral = newI;
					//myPID.Derivative = newD;
				}
			}
			uart0MessageComplete=0;
			//Enable Interrupts again.
			VICIntEnable |= INT_UART0;
		}		
		
		if(sensors_updated)
		{
			sensors_updated=0;
		
			filter.last_time=filter.this_time;
			filter.this_time=millis();	//Get the current number of milliseconds
			//Calculate Interval Time in milliseconds
			filter.interval=filter.this_time-filter.last_time;
			
			//Populate the RwAcc Array
			//filter.fillAccelValues(accelerometer.getX(), accelerometer.getZ());		
			filter.fillAccelValues(xAccelVal, zAccelVal);
			
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
					
					//filter.this_rate=gyro.getX();	//Get the current deg/sec from gyroscope.
					filter.this_rate=xGyroVal;
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
		
			//PIDresult=PIDCalc(&myPID, filter.AccTheta);
			PIDresult=PIDCalc(&myPID, filter.EstTheta);			
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
				(int)filter.EstTheta, 0x67);
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
