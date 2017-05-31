#ifndef _IMU_H_
#define _IMU_H_
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "LFlash.h"

#define   GYRO_SENSITIVITY   131
#define   Y_UPWARD           0
#define   Z_UPWARD           1

class IMU
{
    public:
        IMU();
        void begin();
        void load_gyro_offset();
        void set_gyro_offset();
        void set_offset();
        void print_raw_data();
        void get_raw_data();
        void get_angle();
        void print_angle();
        float dt = 0.01;
        int orientation = Z_UPWARD;
        int sample_number = 10000;
        float angle[3];
        float angle_rate[3];

    private:
        LFlashClass flash;
        MPU6050 mpu6050;
        void get_angle_gyro(float* gyro);
        void get_angle_acce(float* acce);
        float gyro_offset[3];
        float angle_offset = 0.0;
        float raw_gyr[3];
        float raw_acc[3];
        int16_t ax, ay, az, gx, gy, gz;
        float angle_gyro[3];
        float angle_acce = 0.0;
        float compli_factor = 0.9;
};

#endif
