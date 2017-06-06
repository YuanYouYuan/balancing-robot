#include "Controller.h"
#include "IMU.h"
#include "Motor.h"

IMU imu;
Motor motor;
Controller c;

unsigned long timer = 0;
float dt            = 0.01;
float power         = 0.0;

void setup()
{
    Serial.begin(115200);

    c.load_PID_factor();
    c.print_PID_factor();

    imu.begin();
    imu.dt = dt;
    imu.orientation = Y_UPWARD; 
    imu.load_offset();

    motor.enable();
    motor.set_direction(-1);

    Serial.println("Getting started");
    delay(2000);
}

void loop()
{
    timer = millis();
    c.tune_PID_factor();
    imu.get_state();
    power = c.get_PID_feedback(imu.state);
    c.print_PID_feedback();
    while((millis() - timer) < dt * 1000);
}


