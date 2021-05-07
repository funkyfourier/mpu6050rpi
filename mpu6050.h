#ifndef MPU6050
#define MPU6050
#include <time.h>

//Offsets - supply your own here (calculate offsets with getOffsets function)
//     Accelerometer
#define A_OFF_X 14172
#define A_OFF_Y 12727
#define A_OFF_Z -10795
//    Gyroscope
#define G_OFF_X -308
#define G_OFF_Y -157
#define G_OFF_Z 124

//Select the appropriate settings
#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE


#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0b00000000
#endif
#undef ACCEL_RANGE

#define FILTER_GYRO_COEF 0.98f
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

void init_mpu6050();
void stop_mpu6050();
void start_thread();
void get_offsets(void* x, void (*callback)(void*, float, float, float, float, float, float));

struct timespec start,end;

int running;

int f_dev;

float accel_angle[3];
float gyro_angle[3];
float angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

float ax, ay, az, gr, gp, gy, angle_acc_x, angle_acc_y, sgZ;

float dt;

#endif