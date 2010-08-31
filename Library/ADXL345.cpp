/*
	ADXL345 Library
	
	This libary contains functions to interact with the ADXL345 Triple Axis Digital Accelerometer from Analog Devices written for the ATmega328p
	In order to use this libary, define the appropriate pins in the ADXL345.h file

	created 20 Aug 2009
	by Ryan Owens
	http://www.sparkfun.com
 
*/
#include "ADXL345.h"
#include <stdlib.h>
#include <stdio.h>
//#include "LPC214x.h"
#include "LPC21xx_SFE.h"
#include "main.h"
#include "I2C.h"

#define GLOBALOBJECT

ADXL345::ADXL345(int port, char i2c_address)
{
	_i2c_port = port;
	_i2c_address = i2c_address;
	
}

char ADXL345::begin(void)
{
	char values[2];
	configure();
	values[0]=POWER_CTL;
	values[1] = MEASURE;
	//write(DATA_FORMAT, RANGE_1);	//Configure the Accelerometer for +/-8g
	//Set Accel. to Interrupt.  Interrupt will occur on EINT2 pin.
	/*
	ADXL345.write(THRESH_FF, 0x0E);			//Set Accelerometer Threshold to 600 mg
	
	ADXL345.write(TIME_FF, 0x0A);			//Free Fall will trigger after falling for a minimum of  100ms.	
	
	ADXL345.write(BW_RATE, 0x07);			//Set Output Rate to 100 Hz
	ADXL345.write(INT_MAP, ~FREE_FALL);		//Map the Free Fall interrupt to pin INT1; all other interrupts to INT2
	ADXL345.write(INT_ENABLE, FREE_FALL);	//Activate the 'Free Fall' Interrupt
	*/
	return write(values, 2);		//Put the Accelerometer into measurement mode	
}

void ADXL345::powerDown(void)
{

}

char ADXL345::read(char * values, char length){
	
	write(values, 1);	//Set up the i2c address to read from
	return send(_i2c_address, values, READ, length);	
}

char ADXL345::write(char * values, char length){

	return send(_i2c_address, values, WRITE, length);
}

