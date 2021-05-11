#include "m_pd.h"
#include "mpu6050.h"

static t_class *mpu6050rpi_class;

typedef struct _mpu6050rpi
{
    t_object x_obj;
    t_outlet *angle_values_out;
    t_outlet *accel_values_out;
    t_outlet *offset_values_out;
    t_atom values[3];
} t_mpu6050rpi;

void mpu6050rpi_bang(t_mpu6050rpi *x)
{
    float val_x, val_y, val_z;
    
    get_angle(0, &val_x);
    get_angle(1, &val_y);
    get_angle(2, &val_z);
    SETFLOAT(x->values, val_x);
    SETFLOAT(x->values + 1, val_y);
    SETFLOAT(x->values + 2, val_z);
    outlet_list(x->angle_values_out, &s_list, 3, x->values);

    get_accel(&val_x, &val_y, &val_z);
    SETFLOAT(x->values, val_x);
    SETFLOAT(x->values + 1, val_y);
    SETFLOAT(x->values + 2, val_z);
    outlet_list(x->accel_values_out, &s_list, 3, x->values);
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
    SETFLOAT(list, ax_off);
    SETFLOAT(list + 1, ay_off);
    SETFLOAT(list + 2, az_off);
    SETFLOAT(list + 3, gr_off);
    SETFLOAT(list + 4, gp_off);
    SETFLOAT(list + 5, gy_off);
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

void *mpu6050rpi_new(t_symbol *s, int argc, t_atom *argv)
{
    int acc_range = 0, gyro_range = 0;
    if (argc >= 2)
    {
        acc_range = (int)atom_getfloatarg(0, argc, argv);
        gyro_range = (int)atom_getfloatarg(1, argc, argv);
    }

    float filter_coeff = argc == 3 ? atom_getfloatarg(2, argc, argv) : 0.98f;

    init_mpu6050(acc_range, gyro_range, filter_coeff);
    t_mpu6050rpi *x = (t_mpu6050rpi *)pd_new(mpu6050rpi_class);

    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("bang"), gensym("calibrate"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("list"), gensym("set_offsets"));

    x->angle_values_out = outlet_new(&x->x_obj, &s_list);
    x->accel_values_out = outlet_new(&x->x_obj, &s_list);
    x->offset_values_out = outlet_new(&x->x_obj, &s_list);

    return x;
}

void mpu6050rpi_free(t_mpu6050rpi *x)
{
    stop_thread();
}

void mpu6050rpi_start_thread(t_mpu6050rpi *x)
{
    start_thread();
}

void mpu6050rpi_stop_thread(t_mpu6050rpi *x)
{
    stop_thread();
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
        (t_method)mpu6050rpi_start_thread,
        gensym("start"),
        0);

    class_addmethod(
        mpu6050rpi_class,
        (t_method)mpu6050rpi_stop_thread,
        gensym("stop"),
        0);

    class_addmethod(
        mpu6050rpi_class,
        (t_method)mpu6050rpi_set_offsets,
        gensym("set_offsets"),
        A_GIMME,
        0);
}
