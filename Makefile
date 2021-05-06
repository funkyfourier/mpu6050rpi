lib.name = mpu6050rpi

mpu6050rpi.class.sources = mpu6050rpi.c mpu6050.c

ldlibs = -li2c

#ldlibs=-lMPU6050

#cflags = -std=c++11 -DLINK_PLATFORM_LINUX=1 -pthread

include Makefile.pdlibbuilder