#ifndef main_h
#define main_h

#define LED	(1<<15)
#define XBEE_EN	(1<<10)
#define COMM	(1<<12)

#define LED_ON()		IOSET0 = LED
#define LED_OFF()		IOCLR0 = LED

#define XBEEon()		IOSET0 = XBEE_EN
#define XBEEoff()		IOCLR0 = XBEE_EN

#define COMM_ON()		IOSET0 = COMM
#define COMM_OFF()		IOCLR0 = COMM

#define GRAVITY 128
#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

extern int SENSOR_SIGN[9];  //Correct directions x,y,z - gyros, accels, magnetormeter
extern float G_Dt;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

extern long timer;   //general purpuse timer
extern long timer_old;

extern int AN[6]; //array that stores the raw sensor data (gyro X, Y, Z, accel X, Y, Z)
extern int AN_OFFSET[6]; //Array that stores the Offset of the sensors
extern int ACC[3]; //array that store the accelerometers data (may be able to get rid of this.

extern int magnetom_x;
extern int magnetom_y;
extern int magnetom_z;
extern int accel_x;
extern int accel_y;
extern int accel_z;

#define ToDeg(x) (x*57.2957795131)  // *180/pi

#endif
