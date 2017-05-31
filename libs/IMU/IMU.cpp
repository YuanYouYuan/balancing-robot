#include "IMU.h"
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "LFlash.h"

IMU::IMU()
{
    for(int i = 0; i < 3; i++)
    {
        gyro_offset[i] = 0;
        raw_acc[i] = 0;
        raw_gyr[i] = 0;
    }
}

void IMU::begin()
{
    Wire.begin();
    flash.begin();
    mpu6050.initialize();
}

void IMU::get_raw_data()
{
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    if(orientation == Z_UPWARD)
    {
        raw_acc[0] = ax;
        raw_acc[1] = ay;
        raw_acc[2] = az;
        raw_gyr[0] = gx;
        raw_gyr[1] = gy;
        raw_gyr[2] = gz;
    }
    else
    {
        raw_acc[0] = az;
        raw_acc[1] = ax;
        raw_acc[2] = ay;
        raw_gyr[0] = gz;
        raw_gyr[1] = gx;
        raw_gyr[2] = gy;
    }
}

void IMU::print_raw_data()
{
    Serial.print("raw acc:");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(raw_acc[i]);
    }
    Serial.print("\traw gyr: ");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(raw_gyr[i]);
    }
    Serial.println();
}


void IMU::set_offset()
{
    Serial.println("Resetting offset ... ");
    for(int i = 0; i < 3; i++)
        gyro_offset[i] = 0;
    angle_offset = 0;
    float temp_angle_offset = 0;

    for(int i = 0; i < sample_number; i++)
    {
        get_raw_data();

        get_angle_acce(raw_acc);
        temp_angle_offset += angle_acce;

        for(int i = 0; i < 3; i++)
            gyro_offset[i] += raw_gyr[i];
    }
    Serial.print("Gyro offset:");
    for(int i = 0; i < 3; i++)
    {
        gyro_offset[i] /= sample_number;
        Serial.print(" ");
        Serial.print(gyro_offset[i]);
    }
    Serial.print("\tAngle offset: ");
    angle_offset = temp_angle_offset / sample_number;
    Serial.println(angle_offset);
}

void IMU::get_angle()
{
    get_raw_data();

    get_angle_gyro(raw_gyr);
    get_angle_acce(raw_acc);
    
    for(int i = 0; i < 3; i++)
        angle[i] = angle_gyro[i];
    angle[1] = compli_factor * angle[1] + (1 - compli_factor) * angle_acce; 
}

void IMU::get_angle_gyro(float* raw_gyr)
{
    for(int i = 0; i < 3; i++)
    {
        raw_gyr[i] -= gyro_offset[i];
        raw_gyr[i] /= GYRO_SENSITIVITY;
        angle_gyro[i] += raw_gyr[i] * dt;
    }
}

void IMU::get_angle_acce(float* raw_acc)
{
    angle_acce = RAD_TO_DEG * atan2(raw_acc[2], raw_acc[0]);
    angle_acce -= angle_offset;
}

void IMU::print_angle()
{
    Serial.print("Angle_gyro: ");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(angle_gyro[i]);
    }

    Serial.print("\tAngle_acce:");
    Serial.print(angle_acce);

    Serial.print("\tAngle: ");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(angle[i]);
    }
    Serial.println();
}


void IMU::load_gyro_offset()
{
    uint32_t buf_size = sizeof(gyro_offset);
    if(flash.read("factor", "gyro_offset", (uint8_t *)gyro_offset, &buf_size))
        Serial.println("Loading gyro offset succeed");
    else
        Serial.println("Loading gyro offset failed");
}
