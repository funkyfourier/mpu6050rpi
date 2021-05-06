#include "m_pd.h"
#include "mpu6050.h"

static t_class *mpu6050rpi_class;

typedef struct _mpu6050rpi
{
    t_object x_obj;
    t_outlet *angle_x_out;
    t_outlet *angle_y_out;
    t_outlet *angle_z_out;
} t_mpu6050rpi;

void mpu6050rpi_bang(t_mpu6050rpi *x)
{
    post("Hello world !!");
    outlet_float(x->angle_x_out, 212121);
    outlet_float(x->angle_y_out, -212121);
    outlet_float(x->angle_z_out, 0);
}

void *mpu6050rpi_new(void)
{
    init_mpu6050();
    t_mpu6050rpi *x = (t_mpu6050rpi *)pd_new(mpu6050rpi_class);

    x->angle_x_out = outlet_new(&x->x_obj, &s_float);
    x->angle_y_out = outlet_new(&x->x_obj, &s_float);
    x->angle_z_out = outlet_new(&x->x_obj, &s_float);

    return x;
}

void mpu6050rpi_free(t_mpu6050rpi *x)
{
    post("free");
    stop_mpu6050();
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
