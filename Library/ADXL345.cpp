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
#include "LPC214x.h"
#include "main.h"
#include "I2C.h"

#define GLOBALOBJECT

I2C i2c(0);

ADXL345::ADXL345(int port, char i2c_address)
{
	_port = port;
	_i2c_adress = i2c_address;
}

void ADXL345::begin(void)
{
	i2c.begin();
	write(DATA_FORMAT, RANGE_1);	//Configure the Accelerometer for +/-8g
	
	//Set Accel. to Interrupt.  Interrupt will occur on EINT2 pin.
	/*
	ADXL345.write(THRESH_FF, 0x0E);			//Set Accelerometer Threshold to 600 mg
	
	ADXL345.write(TIME_FF, 0x0A);			//Free Fall will trigger after falling for a minimum of  100ms.	
	
	ADXL345.write(BW_RATE, 0x07);			//Set Output Rate to 100 Hz
	ADXL345.write(INT_MAP, ~FREE_FALL);		//Map the Free Fall interrupt to pin INT1; all other interrupts to INT2
	ADXL345.write(INT_ENABLE, FREE_FALL);	//Activate the 'Free Fall' Interrupt
	*/
	write(POWER_CTL, MEASURE);		//Put the Accelerometer into measurement mode	
}

void ADXL345::powerDown(void)
{
	select();
	//SPI0_send(WRITE | Ctrl_Reg1);
	//SPI0_send(~PD);
	unselect();
}

char ADXL345::read(char address){
	address=0x80 | address;
	char register_value=0;
	int spcr_setting=0;
	
	spcr_setting = S0SPCR;	//Save the current SPI Control Register Settings
	S0SPCR  = 0x38;         // Master, no interrupt enable, 8 bits, Active Low SCK pin, CPHA=1	
		
	select();
	delay_ms(1);
	SPI0_send(address);
	register_value=SPI0_recv();
	delay_ms(1);
	unselect();
	
	S0SPCR = spcr_setting;
	return register_value;
}

void ADXL345::write(char address, char value){
	int spcr_setting=0;
	
	spcr_setting = S0SPCR;	//Save the current SPI Control Register Settings
	S0SPCR  = 0x38;         // Master, no interrupt enable, 8 bits, Active Low SCK pin, CPHA=1	
		
	select();
	delay_ms(1);
	SPI0_send(address);
	SPI0_send(value);
	delay_ms(1);
	unselect();
	
	S0SPCR = spcr_setting;
}

inline void ADXL345::unselect(void)
{
	IOSET0 = 1<<_cs_pin;
}

inline void ADXL345::select(void)
{
	IOCLR0 = 1<<_cs_pin;
}
