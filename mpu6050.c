#include "mpu6050.h"

int output_counter;

float dtTotal;

struct OffsetParams
{
    void *x;
    void (*callback)(void *, float, float, float, float, float, float);
};

struct SensorParams
{
    void *x;
    void (*callback)(void *);
};

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
    get_gyro_raw(roll, pitch, yaw);                                       //Store raw values into variables
    *roll = round((*roll - offset_gyro_r) * 1000.0 / gyro_sens) / 1000.0; //Remove the offset and divide by the gyroscope sensetivity (use 1000 and round() to round the value to three decimal places)
    *pitch = round((*pitch - offset_gyro_p) * 1000.0 / gyro_sens) / 1000.0;
    *yaw = round((*yaw - offset_gyro_y) * 1000.0 / gyro_sens) / 1000.0;
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
    get_accel_raw(x, y, z);                                       //Store raw values into variables
    *x = round((*x - offset_acc_x) * 1000.0 / acc_sens) / 1000.0; //Remove the offset and divide by the accelerometer sensetivity (use 1000 and round() to round the value to three decimal places)
    *y = round((*y - offset_acc_y) * 1000.0 / acc_sens) / 1000.0;
    *z = round((*z - offset_acc_z) * 1000.0 / acc_sens) / 1000.0;
}

int get_angle(int axis, float *result)
{
    if (axis >= 0 && axis <= 2)
    {                          //Check that the axis is in the valid range
        *result = angle[axis]; //Get the result
        return 0;
    }
    else
    {
        post("ERR (MPU6050.cpp:getAngle()): 'axis' must be between 0 and 2 (for roll, pitch or yaw)");
        *result = 0; //Set result to zero
        return 1;
    }
}

void *calculate_offsets(void *arg)
{
    running = 0;
    sys_lock();
    post("Calculating the offsets. Please keep the accelerometer level and still. This could take a couple of minutes...");
    sys_unlock();
    struct OffsetParams *offsetParams = arg;

    float ax_off = 0;
    float ay_off = 0;
    float az_off = 0;
    float gr_off = 0;
    float gp_off = 0;
    float gy_off = 0;

    float gyro_off[3]; //Temporary storage
    float accel_off[3];

    for (int i = 0; i < 10000; i++)
    {                                                                                                //Use loop to average offsets
        get_gyro_raw(&gyro_off[0], &gyro_off[1], &gyro_off[2]);                                      //Raw gyroscope values
        gr_off = gr_off + gyro_off[0], gp_off = gp_off + gyro_off[1], gy_off = gy_off + gyro_off[2]; //Add to sum

        get_accel_raw(&accel_off[0], &accel_off[1], &accel_off[2]);                                     //Raw accelerometer values
        ax_off = ax_off + accel_off[0], ay_off = ay_off + accel_off[1], az_off = az_off + accel_off[2]; //Add to sum
    }

    gr_off = gr_off / 10000, gp_off = gp_off / 10000, gy_off = gy_off / 10000; //Divide by number of loops (to average)
    ax_off = ax_off / 10000, ay_off = ay_off / 10000, az_off = az_off / 10000;

    az_off = az_off - acc_sens; //Remove 1g from the value calculated to compensate for gravity)

    offsetParams->callback(offsetParams->x, ax_off, ay_off, az_off, gr_off, gp_off, gy_off);

    free(offsetParams);

    return 0;
}

void get_offsets(void *x, void (*callback)(void *, float, float, float, float, float, float))
{
    struct OffsetParams *offsetParams = (struct OffsetParams *)malloc(sizeof(struct OffsetParams));
    offsetParams->x = x;
    offsetParams->callback = callback;
    pthread_t calc_offset_thread;
    pthread_create(&calc_offset_thread, NULL, calculate_offsets, (void *)offsetParams);
}

void set_offsets(float acc_x, float acc_y, float acc_z, float gyro_r, float gyro_p, float gyro_y)
{
    offset_acc_x = acc_x;
    offset_acc_y = acc_y;
    offset_acc_z = acc_z;
    offset_gyro_r = gyro_r;
    offset_gyro_p = gyro_p;
    offset_gyro_y = gyro_y;
}

static float wrap(float angle_to_wrap, float limit)
{
    while (angle_to_wrap > limit)
        angle_to_wrap -= 2 * limit;
    while (angle_to_wrap < -limit)
        angle_to_wrap += 2 * limit;
    return angle_to_wrap;
}

void stop_thread(pthread_t x_threadid)
{
    running = 0;
    post("pthread_cancel id: %p", x_threadid);
    while (pthread_cancel(x_threadid) < 0){
        post("pthread_cancel");
    }
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

    i2c_smbus_write_byte_data(f_dev, 0x6b, 0b00000000); //Take MPU6050 out of sleep mode - see Register Map

    i2c_smbus_write_byte_data(f_dev, 0x1a, 0b00000011); //Set DLPF (low pass filter) to 44Hz (so no noise above 44Hz will pass through)

    i2c_smbus_write_byte_data(f_dev, 0x19, 0b00000100); //Set sample rate divider (to 200Hz) - see Register Map

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

void *sensor_loop(void *arg)
{
    struct SensorParams *sensorParams = arg;
    clock_gettime(CLOCK_REALTIME, &start);
    while (running > 0)
    {
        pthread_testcancel();
        get_gyro(&gr, &gp, &gy);
        get_accel(&ax, &ay, &az);
        sgZ = (az >= 0) - (az < 0); // allow one angle to go from -180° to +180°

        angle_acc_x = atan2(ay, sgZ * sqrt(az * az + ax * ax)) * RAD_T_DEG; // [-180°,+180°]
        angle_acc_y = -atan2(ax, sqrt(az * az + ay * ay)) * RAD_T_DEG;      // [- 90°,+ 90°]

        angle[0] = wrap(filter_coeff * (angle_acc_x + wrap(angle[0] + gr * dt - angle_acc_x, 180)) + (1.0 - filter_coeff) * angle_acc_x, 180);
        angle[1] = wrap(filter_coeff * (angle_acc_y + wrap(angle[1] + sgZ * gp * dt - angle_acc_y, 90)) + (1.0 - filter_coeff) * angle_acc_y, 90);
        angle[2] += gy * dt;

        dtTotal += dt;

        if (dtTotal >= output_secs)
        {
            sensorParams->callback(sensorParams->x);
            dtTotal = 0;
        }

        clock_gettime(CLOCK_REALTIME, &end);
        dt = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9; //Calculate new dt
        clock_gettime(CLOCK_REALTIME, &start);                                  //Save time to start clock
    }
    sys_lock();
    post("sensor thread stopped");
    sys_unlock();
    free(sensorParams);
    return 0;
}

void start_thread(
    pthread_attr_t thread_attr,
    pthread_t x_threadid,
    void (*callback)(void *),
    void *x)
{
        post("the_threadid: %p", the_threadid);
    if (running == 1){
        post("running");
        stop_thread(x_threadid);
    }

    the_threadid = x_threadid;
        
    running = 1;

    struct SensorParams *sensorParams = (struct SensorParams *)malloc(sizeof(struct SensorParams));
    sensorParams->x = x;
    sensorParams->callback = callback;

    if (pthread_attr_init(&thread_attr) < 0)
    {
        error("threadtemplate: could not launch receive thread");
        return;
    }
    if (pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED) < 0)
    {
        error("threadtemplate: could not launch receive thread");
        return;
    }
    if (pthread_create(&x_threadid, &thread_attr, sensor_loop, (void *)sensorParams) < 0)
    {
        error("threadtemplate: could not launch receive thread");
        return;
    }
    else
    {
        post("sensor thread started");
    }
}

void set_ranges(int acc_range, int gyro_range)
{

    if (acc_range == 1)
    {
        acc_sens = 8192.0;
        acc_config = 0b00001000;
    }
    else if (acc_range == 2)
    {
        acc_sens = 4096.0;
        acc_config = 0b00010000;
    }
    else if (acc_range == 3)
    {
        acc_sens = 2048.0;
        acc_config = 0b00011000;
    }
    else
    {
        acc_sens = 16384.0;
        acc_config = 0b00000000;
    }

    if (gyro_range == 1)
    {
        gyro_sens = 65.5;
        gyro_config = 0b00001000;
    }
    else if (gyro_range == 2)
    {
        gyro_sens = 32.8;
        gyro_config = 0b00010000;
    }
    else if (gyro_range == 3)
    {
        gyro_sens = 16.4;
        gyro_config = 0b00011000;
    }
    else
    {
        gyro_sens = 131.0;
        gyro_config = 0b00000000;
    }
}

void init_mpu6050(int acc_range, int gyro_range, float output_freq, float f_coeff)
{
    post("acc_range: %d, gyro_range: %d, output_freq: %f, f_coeff: %f", acc_range, gyro_range, output_freq, f_coeff);
    filter_coeff = f_coeff;
    output_secs = 1. / output_freq;
    set_ranges(acc_range, gyro_range);
    setup();
}