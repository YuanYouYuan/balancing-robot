#include "IMU.h"

IMU imu;

unsigned long timer;
const float dt = 0.01;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    imu.begin();
    imu.dt = dt;
    /* imu.orientation = Y_UPWARD; */
    delay(1000);
    /* imu.get_raw_data(); */
    /* imu.print_raw_data(); */

    /* imu.reset_offset(); */
    /* imu.write_offset(); */
    /* imu.print_offset(); */

    imu.load_offset();
    imu.print_offset();
    imu.reset_angle();
    delay(2000);
}


void loop()
{
    timer = millis();
    imu.get_state();
    imu.print_state();

    /* Serial.print("Wait dt"); */
    /* while((millis() - timer) < (dt * 1000)) */
        /* Serial.print("."); */
    /* Serial.println(millis() - timer); */
    while((millis() - timer) < (dt * 1000));
}
