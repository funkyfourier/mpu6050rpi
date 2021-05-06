#include "mpu6050.h"
#include "m_pd.h"
#include <pthread.h>
#include <unistd.h>

void* sensor_loop(void* arg){
    while(running == 1){

        post("sensor_loop");
        sleep(1);
    }
    post("thread stopped");
}

void stop_mpu6050(){
    running = 0;
}

void init_mpu6050(){
    post("init_mpu6050");
    running = 1;
    pthread_t sensor_loop_thread;
    pthread_create(&sensor_loop_thread, NULL, sensor_loop, NULL);
}