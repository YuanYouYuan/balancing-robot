#include "LFlash.h"

LFlashClass flash;


float kp, ki, kd;
float kp_step = 0.2;
float ki_step = 0.2;
float kd_step = 0.2;

float write_buffer;
float read_buffer;
uint32_t write_buffer_size = sizeof(write_buffer);
uint32_t read_buffer_size = sizeof(read_buffer);

void setup()
{
    Serial.begin(115200);
    flash.begin();
}

void loop()
{
    if(Serial.available())
    {
        int cmd = Serial.read();
        if(cmd == 'q')
        {
            kp += kp_step;
            factor_print();
        }
        else if(cmd == 'a')
        {
            kp -= kp_step;
            factor_print();
        }
        else if(cmd == 'w')
        {
            ki += ki_step;
            factor_print();
        }
        else if(cmd == 's')
        {
            ki -= ki_step;
            factor_print();
        }
        else if(cmd == 'e')
        {
            kd += kd_step;
            factor_print();
        }
        else if(cmd == 'd')
        {
            kd -= kd_step;
            factor_print();
        }
        else if(cmd == 'x')
            factor_save();
        else if(cmd == 'r')
            factor_read();
    }

}

void factor_print()
{
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(",ki: ");
    Serial.print(ki);
    Serial.print(",kd: ");
    Serial.print(kd);
    Serial.println();
}

void factor_read()
{
    Serial.print("read kp from flash: ");
    Serial.println(kp);
    flash.read("factor", "kp", (uint8_t *)&read_buffer, &read_buffer_size);
    kp = read_buffer;

    Serial.print("read ki from flash: ");
    Serial.println(ki);
    flash.read("factor", "ki", (uint8_t *)&read_buffer, &read_buffer_size);
    ki = read_buffer;

    Serial.print("read kd from flash: ");
    Serial.println(kd);
    flash.read("factor", "kd", (uint8_t *)&read_buffer, &read_buffer_size);
    kd = read_buffer;

    Serial.println("read done");
}

void factor_save()
{
    write_buffer = kp;
    Serial.print("write kp to flash: ");
    Serial.println(kp);
    flash.write("factor", "kp", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    write_buffer = ki;
    Serial.print("write ki to flash: ");
    Serial.println(ki);
    flash.write("factor", "ki", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    write_buffer = kd;
    Serial.print("write kd to flash: ");
    Serial.println(kd);
    flash.write("factor", "kd", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    Serial.println("save done");
}
