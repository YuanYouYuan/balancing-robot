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
        void move(float _power);
        void enable();
        void disable();
        void stop();
        void print_info();
        void set_direction(int _direction);
        float power[2];
    private:
        int ena[2];
        int pin_IN1[2];
        int pin_IN2[2];
        int pin_EN[2];
        int direction;
};

#endif
