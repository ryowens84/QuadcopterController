#ifndef DCM_h
#define DCM_h

void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);

// Euler angles
extern float roll;
extern float pitch;
extern float yaw;


extern float Accel_Vector[3];
extern float Gyro_Vector[3];//Store the SCALED! gyros turn rate in a vector

extern float Omega_Vector[3]; //Corrected Gyro_Vector data
extern float Omega_P[3];//Omega Proportional correction
extern float Omega_I[3];//Omega Integrator
extern float Omega[3];

extern float Gyro_Vector[3];

#endif
