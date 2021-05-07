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
    float angle_x, angle_y, angle_z;
    get_angle(0, &angle_x);
    get_angle(1, &angle_y);
    get_angle(2, &angle_z);
    outlet_float(x->angle_x_out, angle_x);
    outlet_float(x->angle_y_out, angle_y);
    outlet_float(x->angle_z_out, angle_z);
}

void mpu6050rpi_calibrate_callback(
    void *x,
    float ax_off,
    float ay_off,
    float az_off,
    float gr_off,
    float gp_off,
    float gy_off)
{
    t_atom list[6];
    SETFLOAT(list, (t_float)ax_off);
    SETFLOAT(list + 1, (t_float)ay_off);
    SETFLOAT(list + 2, (t_float)az_off);
    SETFLOAT(list + 3, (t_float)gr_off);
    SETFLOAT(list + 4, (t_float)gp_off);
    SETFLOAT(list + 5, (t_float)gy_off);
    t_mpu6050rpi *y = (t_mpu6050rpi *)x;
    outlet_list(y->offset_values_out, &s_list, 6, list);
}

void mpu6050rpi_calibrate(t_mpu6050rpi *x)
{
    get_offsets(x, mpu6050rpi_calibrate_callback);
}

void mpu6050rpi_set_offsets(
    t_mpu6050rpi *x,
    t_symbol *s,
    int argc,
    t_atom *argv)
{
    set_offsets(
        atom_getfloatarg(0, argc, argv),
        atom_getfloatarg(1, argc, argv),
        atom_getfloatarg(2, argc, argv),
        atom_getfloatarg(3, argc, argv),
        atom_getfloatarg(4, argc, argv),
        atom_getfloatarg(5, argc, argv));
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
