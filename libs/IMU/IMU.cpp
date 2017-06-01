#include "IMU.h"
#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "LFlash.h"

IMU::IMU()
{
    for(int i = 0; i < STATE_LIST; i++)
        for(int j = 0; j < 3; j++)
            state_list[i][j] = 0;
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

void IMU::get_state()
{
    get_angle();

    state[1] = (angle[1] - state[0]) / dt;
    state[0] = angle[1];
    state[2] += angle[1] * dt;
    smooth_state();
}

void IMU::smooth_state()
{
    for(int i = 0; i < STATE_LIST - 1; i++)
        for(int j = 0; j < 3; j++)
            state_list[i+1][j] = state_list[i][j];

    for(int j = 0; j < 3; j++)
        state_list[0][j] = state[j];

    float temp_state[3] = {0, 0, 0};
    for(int i = 0; i < STATE_LIST; i++)
        for(int j = 0; j < 3; j++)
            temp_state[j] += state_list[i][j] * (STATE_LIST - i);

    for(int j = 0; j < 3; j++)
    {
        state[j] = temp_state[j] * smooth_weight;
        state_list[0][j] = state[j];
    }
}

void IMU::print_state()
{
    Serial.print("angle: ");
    Serial.print(state[0], 4);
    Serial.print("\tangle rate: ");
    Serial.print(state[1], 4);
    Serial.print("\tangle summ: ");
    Serial.print(state[2], 4);
    Serial.println();
}


void IMU::reset_offset()
{
    Serial.print("Resetting offset...");
    for(int i = 0; i < 3; i++)
        gyro_offset[i] = 0.0;
    angle_offset = 0;
    float temp_angle_offset = 0;

    for(int i = 0; i < sample_number; i++)
    {
        get_raw_data();

        get_acce_angle(raw_acc);
        temp_angle_offset += acce_angle;

        for(int i = 0; i < 3; i++)
            gyro_offset[i] += raw_gyr[i];
    }

    for(int i = 0; i < 3; i++)
        gyro_offset[i] /= sample_number;

    angle_offset = temp_angle_offset / sample_number;

    Serial.println("done");
}

void IMU::print_offset()
{
    Serial.print("Gyro offset:");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(gyro_offset[i]);
    }
    Serial.print("\tAngle offset: ");
    Serial.println(angle_offset);
}

void IMU::get_angle()
{
    get_raw_data();

    get_gyro_angle(raw_gyr);
    get_acce_angle(raw_acc);
    
    for(int i = 0; i < 3; i++)
        angle[i] = gyro_angle[i];
    angle[1] = compli_factor * angle[1] + (1 - compli_factor) * acce_angle; 
}

void IMU::write_offset()
{
    Serial.print("Write offset to flash...");
    flash.write("offset" , "gyro"  , LFLASH_RAW_DATA , (uint8_t*)gyro_offset   , (uint32_t)sizeof(gyro_offset));
    flash.write("offset" , "angle" , LFLASH_RAW_DATA , (uint8_t*)&angle_offset , (uint32_t)sizeof(angle_offset));
    Serial.println("done");
}

void IMU::get_gyro_angle(float* raw_gyr)
{
    for(int i = 0; i < 3; i++)
    {
        raw_gyr[i] -= gyro_offset[i];
        raw_gyr[i] /= GYRO_SENSITIVITY;
        gyro_angle[i] += raw_gyr[i] * dt;
    }
}

void IMU::get_acce_angle(float* raw_acc)
{
    acce_angle = RAD_TO_DEG * atan2(raw_acc[2], raw_acc[0]);
    acce_angle -= angle_offset;
}

void IMU::print_angle()
{
    Serial.print("Gyro angle: ");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(gyro_angle[i]);
    }

    Serial.print("\tAcce angle: ");
    Serial.print(acce_angle);

    Serial.print("\tAngle: ");
    for(int i = 0; i < 3; i++)
    {
        Serial.print(" ");
        Serial.print(angle[i]);
    }
    Serial.println();
}


void IMU::load_offset()
{
    Serial.print("Load offset from flash...");
    uint32_t buf_size;
    buf_size = sizeof(gyro_offset);
    flash.read("offset", "gyro", (uint8_t *)gyro_offset, &buf_size);
    buf_size = sizeof(angle_offset);
    flash.read("offset", "angle", (uint8_t *)&angle_offset, &buf_size);
    Serial.println("done");
}
