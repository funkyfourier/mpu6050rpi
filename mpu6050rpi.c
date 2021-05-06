#include "m_pd.h"
#include "mpu6050.h"

static t_class *mpu6050rpi_class;

typedef struct _mpu6050rpi
{
    t_object x_obj;
    t_outlet *angle_x_out;
    t_outlet *angle_y_out;
    t_outlet *angle_z_out;
    t_outlet *offset_values_out;
} t_mpu6050rpi;

void mpu6050rpi_bang(t_mpu6050rpi *x)
{
    post("Hello world !!");
    outlet_float(x->angle_x_out, 212121);
    outlet_float(x->angle_y_out, -212121);
    outlet_float(x->angle_z_out, 0);
}

void mpu6050rpi_calibrate(t_mpu6050rpi *x)
{
    post("mpu6050rpi_calibrate");
    t_atom list[6];
    SETFLOAT(list, (t_float)1);
    SETFLOAT(list+1, (t_float)2);
    SETFLOAT(list+2, (t_float)3);
    SETFLOAT(list+3, (t_float)4);
    SETFLOAT(list+4, (t_float)5);
    SETFLOAT(list+5, (t_float)6);
    outlet_list(x->offset_values_out, &s_list, 6, list);
}

void mpu6050rpi_set_offsets(
    t_mpu6050rpi *x,
    t_floatarg a_offset_x,
    t_floatarg a_offset_y,
    t_floatarg a_offset_z,
    t_floatarg g_offset_x,
    t_floatarg g_offset_y,
    t_floatarg g_offset_z)
{
    post("a_offset_x: %f, a_offset_y: %f", a_offset_x, a_offset_y);
}

void *mpu6050rpi_new(void)
{
    init_mpu6050();
    t_mpu6050rpi *x = (t_mpu6050rpi *)pd_new(mpu6050rpi_class);

    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("bang"), gensym("calibrate"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("list"), gensym("set_offsets"));

    x->angle_x_out = outlet_new(&x->x_obj, &s_float);
    x->angle_y_out = outlet_new(&x->x_obj, &s_float);
    x->angle_z_out = outlet_new(&x->x_obj, &s_float);
    x->offset_values_out = outlet_new(&x->x_obj, &s_list);

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

    class_addmethod(
        mpu6050rpi_class,
        (t_method)mpu6050rpi_calibrate,
        gensym("calibrate"),
        0);

    class_addmethod(
        mpu6050rpi_class,
        (t_method)mpu6050rpi_set_offsets,
        gensym("set_offsets"),
        A_GIMME,
        0);
}
