#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 7
bool blinkState = false;

const int angle_list_number = 5;
const float K  = 1;
float gyro_angle = 0;
float acce_angle = 0;
float angle_list[angle_list_number];
float time, time_pre, time_step;
float offset = 0;

void setup() {
    Wire.begin();
    Serial.begin(115200);

    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED_PIN, OUTPUT);

    time = millis();
}

void loop() {

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // read raw accel/gyro measurements from device

    // display tab-separated accel/gyro x/y/z values
    //Serial.print("a/g:\t");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print("\n");

	//acce_angle = (180/3.141593) * atan((float)sqrt(ax*ax+az*az)/ax);
    //double scale_ax = (double)ax/az;
    //double angle = (180/3.141593) * atan(scale_ax/sqrt(scale_ax * scale_ax + 1));

    //time_pre = time;
	//time = millis();
	//time_step = (time - time_pre)/1000;
	//gyro_angle += gy * time_step;


    //Serial.print(angle); Serial.print(",");
    //Serial.println(gyro_angle);
}
