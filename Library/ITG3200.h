#ifndef ITG3200_h
#define ITG3200_h

#include "LPC21xx_SFE.h"
#include "I2C.h"

class ITG3200: public I2C
{
	public:
		ITG3200(int port, char i2c_address);
		char begin(void);
		char read(char * values, char length);
		char write(char * values, char length);
	private:
		char _i2c_address;
};

/* ************************ Register map for the ITG3200 ****************************/
#define ITG_ADDR	0xD0

#define WHO_AM_I	0x00
#define SMPLRT_DIV	0x15
#define	DLPF_FS		0x16
#define INT_CFG		0x17
#define INT_STATUS	0x1A
#define	TEMP_OUT_H	0x1B
#define	TEMP_OUT_L	0x1C
#define GYRO_XOUT_H	0x1D
#define	GYRO_XOUT_L	0x1E
#define GYRO_YOUT_H	0x1F
#define GYRO_YOUT_L	0x20
#define GYRO_ZOUT_H	0x21
#define GYRO_ZOUT_L	0x22
#define	PWR_MGM		0x3E

#endif
