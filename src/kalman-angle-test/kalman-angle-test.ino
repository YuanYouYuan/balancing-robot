#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Kalman.h"
#include "LFlash.h"
MPU6050 IMU;
LFlashClass flash;

#define DEBUG_ANGLE
//#define DEBUG_RAW_DATA

#define GYRO_SENSITIVITY 131
#define SAMPLE_NUMBER 20
#define PIN_BTN 6


int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kal;
float angle = 0;
float angle_acce = 0;
float angle_list[SAMPLE_NUMBER];
float dt, time, time_pre;

float gy_offset = 0;
float read_buffer;
uint32_t read_buffer_size = sizeof(read_buffer);

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    IMU.initialize();
    flash.begin();

    for(int i = 0; i < SAMPLE_NUMBER; i++)
        angle_list[i] = 0;

    Serial.print("read gy_offset from flash: ");
    Serial.println(gy_offset);
    flash.read("factor", "gy_offset", (uint8_t *)&read_buffer, &read_buffer_size);
    gy_offset = read_buffer;
}

void loop()
{
    Serial.print(get_angle(), 4);
    Serial.print(", ");
    Serial.println(angle_correction(), 4);
}

float get_angle()
{
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    time_pre = time;
    time = millis();
    dt = (time - time_pre)/1000.0;
    angle_acce = (180/3.141592) * atan2((float)az, (float)ax) -90;
    kal.setAngle(angle_acce);
    angle = kal.getAngle(angle_acce, (float)gy/GYRO_SENSITIVITY, dt);
    return angle;
}

float angle_correction()
{
    for(int i = 0; i < SAMPLE_NUMBER - 1; i++)
        angle_list[i] = angle_list[i+1];
    angle_list[SAMPLE_NUMBER-1] = angle;
    float angle_mean = 0;
    for(int i = 0; i < SAMPLE_NUMBER; i++)
        angle_mean += angle_list[i];
    angle_mean /= SAMPLE_NUMBER;
    return angle_mean;
}
