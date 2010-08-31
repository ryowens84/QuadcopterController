/*
	HMC5843 Library
	
	This libary contains functions to interact with the HMC5843
	I2C.h must be included as the I2C class is inherited

	created 8/30/10
	by Ryan Owens
	http://www.sparkfun.com
 
*/
#include "HMC5843.h"

#include <stdlib.h>
#include <stdio.h>
#include "LPC21xx_SFE.h"
#include "main.h"
#include "I2C.h"

#define GLOBALOBJECT

HMC5843::HMC5843(int port, char i2c_address)
{
	_i2c_port = port;
	_i2c_address = i2c_address;
	
}

char HMC5843::begin(void)
{
	char values[2];
	configure();
	
	values[0]=CONFIG_REGA;
	values[1]=0x14;
	while(!write(values,2));
	
	values[0]=MODE_REG;	//Set the HMC module to read the Configuration Reg. A	
	values[1]=0x00;		//Set the HMC to continuous conversion mode
	return write(values, 2);	//Write the new data to the HMC register.
}


char HMC5843::read(char * values, char length){
	
	write(values, 1);	//Set up the i2c address to read from
	return send(_i2c_address, values, READ, length);	
}

char HMC5843::write(char * values, char length){

	return send(_i2c_address, values, WRITE, length);
}

