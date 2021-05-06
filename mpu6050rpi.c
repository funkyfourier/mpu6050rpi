#include "m_pd.h"

static t_class *mpu6050rpi_class;

typedef struct _mpu6050rpi
{
    t_object x_obj;
} t_mpu6050rpi;

void mpu6050rpi_bang(t_mpu6050rpi *x)
{
    post("Hello world !!");
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
