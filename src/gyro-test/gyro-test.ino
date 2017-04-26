#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const int s = 131;

void setup() 
   Wire.begin();
    Serial.begin(115200);
    accelgyro.initialize();
}

void loop() 
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //Serial.print(ax); Serial.print(",");
    //Serial.print(ay); Serial.print(",");
    //Serial.print(az); Serial.print(",");
    Serial.print((float)gx/s, 4); Serial.print(",");
    Serial.print((float)gy/s, 4); Serial.print(",");
    Serial.print((float)gz/s, 4); Serial.print(",");
	Serial.println(millis());
}
