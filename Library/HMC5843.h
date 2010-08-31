#ifndef HMC5843_h
#define HMC5843_h

#include "LPC21xx_SFE.h"
#include "I2C.h"

class HMC5843: public I2C
{
	public:
		HMC5843(int port, char i2c_address);
		char begin(void);
		char read(char * values, char length);
		char write(char * values, char length);
	private:
		char _i2c_address;
};

/* ************************ Register map for the HMC5843 ****************************/
//I2C Address for the HMC5843
#define HMC_ADDR	0x3C

//HMC5843 Register Addresses
#define	CONFIG_REGA	0
#define	CONFIG_REGB	1
#define	MODE_REG	2
#define	DATA_OUT_X_H	3
#define DATA_OUT_X_L	4
#define DATA_OUT_Y_H	5
#define	DATA_OUT_Y_L	6
#define	DATA_OUT_Z_H	7
#define	DATA_OUT_Z_L	8
#define	STATUS_REG		9
#define ID_REGA		10
#define	ID_REGB		11
#define	ID_REGC		12

#endif
