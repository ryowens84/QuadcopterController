/*
	HMC5843 Library
	
	This libary contains functions to interact with the HMC5843
	I2C.h must be included as the I2C class is inherited

	created 8/30/10
	by Ryan Owens
	http://www.sparkfun.com
	
	Example Code:
	char values[6];
	char status=0;
	
	while(!compass.begin()){
		rprintf("Begin Failed\n\r");
		delay_ms(100);
	}
	values[0]=0x02;
	values[1]=0x00;
	while(!compass.write(values, 2));
	values[0]=0x02;
	while(!compass.read(values, 1));
	rprintf("Mode: %02x\n\r", values[0]);
	values[0]=ID_REGA;
	while(!compass.read(values, 1));
	rprintf("ID: %02x\n\r", values[0]);	
	delay_ms(1000);
	status=0;
	while(1){
		while(!(status & (1<<0))){
			values[0]=STATUS_REG;
			while(!compass.read(values,  1));	//Read the Status Register
			status=values[0];
			//rprintf("Status: %02x\n\r", status);
		}
		status=0;
		values[0]=DATA_OUT_X_H;
		while(!compass.read(values, 6));
		for(int i=0; i<3; i++)
			rprintf("%02x%02x\t", values[i*2], values[i*2+1]);
		rprintf("\n\r");
	}		
 
*/
#include "HMC5843.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "LPC21xx_SFE.h"
#include "main.h"

#define GLOBALOBJECT

cHMC5843 compass(0, HMC_ADDR);

cHMC5843::cHMC5843(int port, char i2c_address)
{
	_i2c_port = port;
	_i2c_address = i2c_address;
	
}

void cHMC5843::begin(void)
{
	//char values[2];
	configure();
	
	//Configure the compass for 50 Hz Update Rage
	values[0]=CONFIG_REGA;
	values[1]=CONFIG_REGA_DO2|CONFIG_REGA_DO1;
	write(values,2);
	
	//Compass will use default gain
	//Don't change CONFIG_REGB
	
	//Configure compass for continuous conversion
	values[0]=MODE_REG;	
	values[1]=0x00;		//Set the HMC to continuous conversion mode
	write(values, 2);	//Write the new data to the HMC register.
}


char cHMC5843::read(char * value, char length){
	
	write(value, 1);	//Set up the i2c address to read from
	return send(_i2c_address, value, READ, length);	
}

char cHMC5843::write(char * value, char length){

	return send(_i2c_address, value, WRITE, length);
}

char cHMC5843::update(void)
{
	values[0]=DATA_OUT_X_H;
	if(read(values, 6))
	{
		xc = (int16_t)((values[0]<<8)|values[1]);
		yc = (int16_t)((values[2]<<8)|values[3]);
		zc = (int16_t)((values[4]<<8)|values[5]);
	}
	else return 0;
	return 1;
}

float cHMC5843::getX(void)
{
	xc = xc/2048;
	return xc;
}

float cHMC5843::getY(void)
{
	yc = yc/2048;
	return yc;
}

float cHMC5843::getZ(void)
{
	zc = zc/2048;
	return zc;
}

float cHMC5843::getHeading(void)
{
	getX();
	getY();
	getZ();
	
	if(yc > 0)
	{
		heading = 90-(atan2(xc,yc))*(180/3.14);
	}
	else if(yc < 0)
	{
		heading = 270-(atan2(xc,yc))*(180/3.14);
	}
	else
	{
		if(xc > 0)
		{
			heading = 180.0;
		}
		else
		{
			heading = 0.0;
		}
	}
	return heading;
	
}
