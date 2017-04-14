#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


#define GYRO_SENSITIVITY 131

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define LED_PIN 7
bool blinkState = false;

const int angle_list_number = 5;
const double gy_mean = 235.90;
const double complimentary_const = 0.5;

double gyro_angle = 0;
double acce_angle = 0;
double angle_list[angle_list_number];
double time, time_pre, time_step;
double offset = 0;

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

    print_raw_data();
    Serial.print("acce_angle: ");
    Serial.println(get_angle(ax, az, gy));
    

    // read raw accel/gyro measurements from device

    // display tab-separated accel/gyro x/y/z values
    //Serial.print("a/g:\t");

	//acce_angle = (180/3.141593) * atan((double)sqrt(ax*ax+az*az)/ax);
    //double scale_ax = (double)ax/az;
    //double angle = (180/3.141593) * atan(scale_ax/sqrt(scale_ax * scale_ax + 1));



    //Serial.print(angle); Serial.print(",");
    //Serial.println(gyro_angle);
}

void print_raw_data()
{
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print("\n");
}

double get_angle(int16_t ax, int16_t az, int16_t gy)
{
    double angle;
    acce_angle = 90.0 + (180/3.141592) * atan2((double)az, (double)ax);
    time_pre = time;
	time = millis();
	time_step = (time - time_pre)/1000;
	gyro_angle += (double)(gy - gy_mean)/GYRO_SENSITIVITY * time_step;
    angle = acce_angle * complimentary_const + gyro_angle * (1 - complimentary_const);
    return angle;
}
