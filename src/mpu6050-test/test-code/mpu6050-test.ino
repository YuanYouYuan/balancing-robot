#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 13
bool blinkState = false;

const int angle_list_number = 5;
const float K  = 1;
float gyro_angle = 0;
float acce_angle = 0;
float angle_list[angle_list_number];
float time, time_pre, time_step;
float offset = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(LED_PIN, OUTPUT);

    time = millis();
    for(int i = 0; i < angle_list_number; i++)
		angle_list[i] = 0.0;
}

void loop() {
    // read raw accel/gyro measurements from device

    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	acce_angle = (180/3.141593) * atan((float)ay/ax);
    Serial.println(acce_angle);
    delay(500);
}

float get_angle()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    time_pre = time;
	time = millis();
	time_step = (time - time_pre)/1000;
	gyro_angle += gz*time_step;
	acce_angle = (180/3.141593) * atan(ay/ax);
	for(int i = 0; i < angle_list_number-1; i++)
		angle_list[i] = angle_list[i+1];
	angle_list[angle_list_number-1] = K * acce_angle + (1-K) * gyro_angle;
	float mean_angle;
	mean_angle = 0.0;
	for(int i = 0; i < angle_list_number; i++)
		mean_angle += angle_list[i];
	mean_angle /= 5;
	mean_angle -= offset;
    return mean_angle;
}
