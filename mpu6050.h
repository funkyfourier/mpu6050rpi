#ifndef MPU6050
#define MPU6050

#include <time.h>
#include <math.h>
#include "m_pd.h"
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdlib.h>

#define FILTER_GYRO_COEF 0.98f
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

void init_mpu6050(int acc_range, int gyro_range, float f_coeff);
void stop_thread();
void start_thread();
void get_offsets(void* x, void (*callback)(void*, float, float, float, float, float, float));
void set_offsets(float acc_x, float acc_y, float acc_z, float gyro_r, float gyro_p, float gyro_y);
int get_angle(int axis, float *result);

float offset_acc_x;
float offset_acc_y;
float offset_acc_z;
float offset_gyro_r;
float offset_gyro_p;
float offset_gyro_y;

uint8_t acc_config;
uint8_t gyro_config;
float acc_sens;
float gyro_sens;

float filter_coeff;

struct timespec start,end;

int running;

int f_dev;

float accel_angle[3];
float gyro_angle[3];
float angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

float ax, ay, az, gr, gp, gy, angle_acc_x, angle_acc_y, sgZ;

float dt;

#endif