#ifndef _IMU_H_
#define _IMU_H_
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "LFlash.h"

#define GYRO_SENSITIVITY 131
#define STATE_LIST       5
#define Y_UPWARD         0
#define Z_UPWARD         1

class IMU
{
    public:
        IMU();
        void begin();

        void get_raw_data();
        void print_raw_data();
        void get_angle();
        void print_angle();

        void reset_offset();
        void write_offset();
        void load_offset();
        void print_offset();

        void get_state();
        void print_state();

        float dt          = 0.01;
        int orientation   = Z_UPWARD;
        int sample_number = 10000;
        float angle[3]    = {0, 0, 0};
        float state[3]    = {0, 0, 0};

    private:
        LFlashClass flash;
        MPU6050 mpu6050;
        void get_gyro_angle(float* gyro);
        void get_acce_angle(float* acce);
        void smooth_state();

        int16_t ax, ay, az, gx, gy, gz;
        float raw_gyr[3]     = {0, 0, 0};
        float raw_acc[3]     = {0, 0, 0};
        float gyro_offset[3] = {0, 0, 0};
        float angle_offset   = 0.0;

        float gyro_angle[3]  = {0, 0, 0};
        float acce_angle     = 0.0;
        float state_list[STATE_LIST][3];
        float smooth_weight = 2.0 / (STATE_LIST * (STATE_LIST + 1));
        float compli_factor  = 0.5;
};

#endif
