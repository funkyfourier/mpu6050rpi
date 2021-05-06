#include "m_pd.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

static t_class *mpu6050rpi_class;

typedef struct _mpu6050rpi
{
    t_object x_obj;
} t_mpu6050rpi;

void mpu6050rpi_bang(t_mpu6050rpi *x)
{
    post("Hello world !!");

    int MPU6050_addr = 0x68;

    int f_dev = open("/dev/i2c-1", O_RDWR);

    if (f_dev < 0)
    { //Catch errors
        post("ERROR: Failed to open /dev/i2c-1. Please check that I2C is enabled with raspi-config");
    }

    int status = ioctl(f_dev, I2C_SLAVE, MPU6050_addr);

    if (status < 0) {
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

void *mpu6050rpi_new(void)
{
    t_mpu6050rpi *x = (t_mpu6050rpi *)pd_new(mpu6050rpi_class);
    return x;
}

void mpu6050rpi_free(t_mpu6050rpi *x)
{
}

void mpu6050rpi_setup(void)
{
    mpu6050rpi_class = class_new(
        gensym("mpu6050rpi"),
        (t_newmethod)mpu6050rpi_new,
        (t_method)mpu6050rpi_free,
        sizeof(t_mpu6050rpi),
        CLASS_DEFAULT,
        A_GIMME,
        0);

    class_addbang(mpu6050rpi_class, mpu6050rpi_bang);
}
