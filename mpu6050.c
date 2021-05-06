#include "mpu6050.h"
#include "m_pd.h"
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <math.h>
#include <stdlib.h>

void get_gyro_raw(float *roll, float *pitch, float *yaw)
{
    int16_t X = i2c_smbus_read_byte_data(f_dev, 0x43) << 8 | i2c_smbus_read_byte_data(f_dev, 0x44); //Read X registers
    int16_t Y = i2c_smbus_read_byte_data(f_dev, 0x45) << 8 | i2c_smbus_read_byte_data(f_dev, 0x46); //Read Y registers
    int16_t Z = i2c_smbus_read_byte_data(f_dev, 0x47) << 8 | i2c_smbus_read_byte_data(f_dev, 0x48); //Read Z registers
    *roll = (float)X;                                                                               //Roll on X axis
    *pitch = (float)Y;                                                                              //Pitch on Y axis
    *yaw = (float)Z;                                                                                //Yaw on Z axis
}

void get_gyro(float *roll, float *pitch, float *yaw)
{
    get_gyro_raw(roll, pitch, yaw);                                 //Store raw values into variables
    *roll = round((*roll - G_OFF_X) * 1000.0 / GYRO_SENS) / 1000.0; //Remove the offset and divide by the gyroscope sensetivity (use 1000 and round() to round the value to three decimal places)
    *pitch = round((*pitch - G_OFF_Y) * 1000.0 / GYRO_SENS) / 1000.0;
    *yaw = round((*yaw - G_OFF_Z) * 1000.0 / GYRO_SENS) / 1000.0;
}

void get_accel_raw(float *x, float *y, float *z)
{
    int16_t X = i2c_smbus_read_byte_data(f_dev, 0x3b) << 8 | i2c_smbus_read_byte_data(f_dev, 0x3c); //Read X registers
    int16_t Y = i2c_smbus_read_byte_data(f_dev, 0x3d) << 8 | i2c_smbus_read_byte_data(f_dev, 0x3e); //Read Y registers
    int16_t Z = i2c_smbus_read_byte_data(f_dev, 0x3f) << 8 | i2c_smbus_read_byte_data(f_dev, 0x40); //Read Z registers
    *x = (float)X;
    *y = (float)Y;
    *z = (float)Z;
}

void get_accel(float *x, float *y, float *z)
{
    get_accel_raw(x, y, z);                                    //Store raw values into variables
    *x = round((*x - A_OFF_X) * 1000.0 / ACCEL_SENS) / 1000.0; //Remove the offset and divide by the accelerometer sensetivity (use 1000 and round() to round the value to three decimal places)
    *y = round((*y - A_OFF_Y) * 1000.0 / ACCEL_SENS) / 1000.0;
    *z = round((*z - A_OFF_Z) * 1000.0 / ACCEL_SENS) / 1000.0;
}

struct OffsetParams
{
    float *ax_off;
    float *ay_off;
    float *az_off;
    float *gr_off;
    float *gp_off;
    float *gy_off;
};

void *calculate_offsets(void *arg)
{
    struct OffsetParams *offsetParams = arg;

    float* ax_off = offsetParams->ax_off;
    float* ay_off = offsetParams->ay_off;
    float* az_off = offsetParams->az_off;
    float* gr_off = offsetParams->gr_off;
    float* gp_off = offsetParams->gp_off;
    float* gy_off = offsetParams->gy_off;
    
    float gyro_off[3]; //Temporary storage
    float accel_off[3];

    *gr_off = 0, *gp_off = 0, *gy_off = 0; //Initialize the offsets to zero
    *ax_off = 0, *ay_off = 0, *az_off = 0; //Initialize the offsets to zero

    for (int i = 0; i < 10000; i++)
    {                                                                                                      //Use loop to average offsets
        get_gyro_raw(&gyro_off[0], &gyro_off[1], &gyro_off[2]);                                            //Raw gyroscope values
        *gr_off = *gr_off + gyro_off[0], *gp_off = *gp_off + gyro_off[1], *gy_off = *gy_off + gyro_off[2]; //Add to sum

        get_accel_raw(&accel_off[0], &accel_off[1], &accel_off[2]);                                           //Raw accelerometer values
        *ax_off = *ax_off + accel_off[0], *ay_off = *ay_off + accel_off[1], *az_off = *az_off + accel_off[2]; //Add to sum
    }

    *gr_off = *gr_off / 10000, *gp_off = *gp_off / 10000, *gy_off = *gy_off / 10000; //Divide by number of loops (to average)
    *ax_off = *ax_off / 10000, *ay_off = *ay_off / 10000, *az_off = *az_off / 10000;

    *az_off = *az_off - ACCEL_SENS; //Remove 1g from the value calculated to compensate for gravity)*/
}

void get_offsets(float *ax_off, float *ay_off, float *az_off, float *gr_off, float *gp_off, float *gy_off)
{
    struct OffsetParams *offsetParams = (struct OffsetParams *)malloc(sizeof(struct OffsetParams));
    offsetParams->ax_off = ax_off;
    offsetParams->ay_off = ay_off;
    offsetParams->az_off = az_off;
    offsetParams->gr_off = gr_off;
    offsetParams->gp_off = gp_off;
    offsetParams->gy_off = gy_off;
    pthread_t calculate_offsets_thread;
    pthread_create(&calculate_offsets_thread, NULL, calculate_offsets, (void *)offsetParams);
    pthread_join(calculate_offsets_thread, NULL);
}

static float wrap(float angle_to_wrap, float limit)
{
    while (angle_to_wrap > limit)
        angle_to_wrap -= 2 * limit;
    while (angle_to_wrap < -limit)
        angle_to_wrap += 2 * limit;
    return angle_to_wrap;
}

void *sensor_loop(void *arg)
{
    clock_gettime(CLOCK_REALTIME, &start);
    while (running > 0)
    {
        get_gyro(&gr, &gp, &gy);
        get_accel(&ax, &ay, &az);
        sgZ = (az >= 0) - (az < 0); // allow one angle to go from -180° to +180°

        angle_acc_x = atan2(ay, sgZ * sqrt(az * az + ax * ax)) * RAD_T_DEG; // [-180°,+180°]
        angle_acc_y = -atan2(ax, sqrt(az * az + ay * ay)) * RAD_T_DEG;      // [- 90°,+ 90°]

        angle[0] = wrap(FILTER_GYRO_COEF * (angle_acc_x + wrap(angle[0] + gr * dt - angle_acc_x, 180)) + (1.0 - FILTER_GYRO_COEF) * angle_acc_x, 180);
        angle[1] = wrap(FILTER_GYRO_COEF * (angle_acc_y + wrap(angle[1] + sgZ * gp * dt - angle_acc_y, 90)) + (1.0 - FILTER_GYRO_COEF) * angle_acc_y, 90);
        angle[2] += gy * dt;
    }
    post("thread stopped");
    return 0;
}

void stop_mpu6050()
{
    running = 0;
}

void setup()
{
    int MPU6050_addr = 0x68;

    f_dev = open("/dev/i2c-1", O_RDWR);

    if (f_dev < 0)
    { //Catch errors
        post("ERROR: Failed to open /dev/i2c-1. Please check that I2C is enabled with raspi-config");
    }

    int status = ioctl(f_dev, I2C_SLAVE, MPU6050_addr);

    if (status < 0)
    {
        post("ERROR: Could not get I2C bus with adress: %i", MPU6050_addr); //Print error message
    }

    post("status: %d", status);

    i2c_smbus_write_byte_data(f_dev, 0x6b, 0b00000000); //Take MPU6050 out of sleep mode - see Register Map

    i2c_smbus_write_byte_data(f_dev, 0x1a, 0b00000011); //Set DLPF (low pass filter) to 44Hz (so no noise above 44Hz will pass through)

    i2c_smbus_write_byte_data(f_dev, 0x19, 0b00000100); //Set sample rate divider (to 200Hz) - see Register Map

    //TODO Set up different ranges
    uint8_t gyro_config = 0b00000000;
    uint8_t acc_config = 0b00000000;

    i2c_smbus_write_byte_data(f_dev, 0x1b, gyro_config); //Configure gyroscope settings - see Register Map (see MPU6050.h for the GYRO_CONFIG parameter)

    i2c_smbus_write_byte_data(f_dev, 0x1c, acc_config);

    //Set offsets to zero
    i2c_smbus_write_byte_data(f_dev, 0x06, 0b00000000),
        i2c_smbus_write_byte_data(f_dev, 0x07, 0b00000000),
        i2c_smbus_write_byte_data(f_dev, 0x08, 0b00000000),
        i2c_smbus_write_byte_data(f_dev, 0x09, 0b00000000),
        i2c_smbus_write_byte_data(f_dev, 0x0A, 0b00000000),
        i2c_smbus_write_byte_data(f_dev, 0x0B, 0b00000000),
        i2c_smbus_write_byte_data(f_dev, 0x00, 0b10000001),
        i2c_smbus_write_byte_data(f_dev, 0x01, 0b00000001),
        i2c_smbus_write_byte_data(f_dev, 0x02, 0b10000001);
}

void start_thread()
{
    running = 1;
    pthread_t sensor_loop_thread;
    pthread_create(&sensor_loop_thread, NULL, sensor_loop, NULL);
}

void init_mpu6050()
{
    post("init_mpu6050");
    setup();
    start_thread();
}