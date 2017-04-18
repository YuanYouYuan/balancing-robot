#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

long time, time_pre;
long dt;

void setup() {
    Wire.begin();
    Serial.begin(115200);

    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    time = millis();
}

void loop() {

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");

    time_pre = time;
	time = millis();
    dt = time - time_pre;
    Serial.print(dt); Serial.print("\n");
    
}
