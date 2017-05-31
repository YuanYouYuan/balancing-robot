#include "IMU.h"

IMU imu;

unsigned long timer;
const float dt = 0.01;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    imu.begin();
    delay(1000);
    imu.get_raw_data();
    imu.print_raw_data();
    imu.set_offset();
    imu.dt = dt;
    delay(2000);
}


void loop()
{
    timer = millis();
    imu.get_angle();
    imu.print_angle();

    Serial.print("Wait dt");
    while((millis() - timer) < (dt * 1000))
        Serial.print(".");
    Serial.println(millis() - timer);
}
