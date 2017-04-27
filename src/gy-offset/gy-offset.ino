#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "LFlash.h"
MPU6050 IMU;
LFlashClass flash;

int16_t ax, ay, az;
int16_t gx, gy, gz;
float gy_offset = 0;
long sample_number = 10000;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    flash.begin();
    IMU.initialize();

    while(!Serial);
    Serial.println("Start correction ...");
    Serial.println("Please wait ...");

    for(int i = 0; i < sample_number; i++)
    {
        IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        gy_offset += gy;
    }
    gy_offset /= sample_number;
    Serial.println("gy_offset: ") ;
    Serial.println(gy_offset, 4) ;

    float write_buffer = gy_offset;
    uint32_t write_buffer_size = sizeof(write_buffer);
    Serial.print("write gy_offset to flash: ");
    Serial.println(gy_offset);
    flash.write("factor", "gy_offset", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);
    Serial.println("done");
}

void loop()
{
}

