/*
Library for filtering Accelerometer and Gyroscope Values
Based on "A Guide to Using IMU in Embedded Applications" by Starlino
http://starlino.com/imu_guide.html

Rewritten for LPC2148 by Ryan Owens
10/18/10
*/

#include "sensor.h"
#include <stdlib.h>
#include <stdio.h>
#include "LPC21xx_SFE.h"
#include "main.h"
#include <math.h>

#define GLOBALOBJECT

cSensor filter;

cSensor::cSensor(void)
{
	Axz=0;
	gyro_weight=5;
	signRzGyro=0;
	AccTheta=0, EstTheta=0;
	last_time=0, this_time=0;
	interval=0;
	first_run=1;
}

void cSensor::fillAccelValues(float x, float z)
{
	RwAcc[0] = x;
	RwAcc[1] = z;
}

void cSensor::normalizeVector(double * vector)
{
	double R;
	R = sqrt(vector[0]*vector[0] + vector[1]*vector[1]);
	vector[0] /= R;
	vector[1] /= R;	
}
