#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "Arduino.h"

#define MOTOR_A 0
#define MOTOR_B 1

class Motor
{
    public:
        Motor();
        Motor(int pin_EN_A, int pin_IN1_A, int pin_IN2_A, int pin_EN_B, int pin_IN1_B, int pin_IN2_B);
        void move(float power);
        void move_two(float power_A, float power_B);
        void enable();
        void disable();
        void stop();
        void print_info();
        void set_direction(int direction);
    private:
        struct m
        {
            bool enable;
            float power;
            int pin_IN1;
            int pin_IN2;
            int pin_EN;
        }
        struct motors m[2];
        int direction;
};

#endif
