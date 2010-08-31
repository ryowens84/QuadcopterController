/*
	ITG3200 Library
	
	This libary contains functions to interact with the ITG3200
	I2C.h must be included as the I2C class is inherited

	created 8/30/10
	by Ryan Owens
	http://www.sparkfun.com
 
*/
#include "ITG3200.h"

#include <stdlib.h>
#include <stdio.h>
#include "LPC21xx_SFE.h"
#include "main.h"
#include "I2C.h"

#define GLOBALOBJECT

ITG3200::ITG3200(int port, char i2c_address)
{
	_i2c_port = port;
	_i2c_address = i2c_address;
	
}

char ITG3200::begin(void)
{
	char values[2];
	configure();
	
	//values[0]=;	
	//values[1]=;		
	//return write(values, 2);	//Write the new data to the HMC register.
	return 1;
}


char ITG3200::read(char * values, char length){
	
	write(values, 1);	//Set up the i2c address to read from
	return send(_i2c_address, values, READ, length);	
}

char ITG3200::write(char * values, char length){

	return send(_i2c_address, values, WRITE, length);
}

