#include "LFlash.h"

LFlashClass flash;

#define DATA_LENGTH 10
float data[DATA_LENGTH];
long time[DATA_LENGTH];

void setup()
{

    Serial.begin(115200);
    flash.begin();

    while(!Serial);
    Serial.println("generate data randomly");
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        data[i] = random(1, 100);
        time[i] = millis();
        delay(10);
    }

    Serial.println("write data into memory");
    write_data_channel(data, "data");
    write_data_time(time);

    Serial.println("read data from memory");
    read_data_channel("data");

    Serial.println("original data");
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.println("read time from memory");
    read_data_time();

    Serial.println("original time");
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        Serial.print(time[i]);
        Serial.print(" ");
    }
    Serial.println();
}

void loop()
{

}


void read_data_channel(const char* property_name)
{
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        float read_buffer;
        uint32_t read_buffer_size = sizeof(read_buffer);
        flash.read("log_"+i, property_name, (uint8_t *)&read_buffer, &read_buffer_size);
        Serial.print(read_buffer);
        Serial.print(" ");
    }
    Serial.println();
}

void read_data_time()
{
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        long read_buffer;
        uint32_t read_buffer_size = sizeof(read_buffer);
        flash.read("log_"+i, "time", (uint8_t *)&read_buffer, &read_buffer_size);
        Serial.print(read_buffer);
        Serial.print(" ");
    }
    Serial.println();
}

void write_data_channel(float* data_channel, const char* property_name)
{
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        Serial.print("write ");
        Serial.print(i);
        Serial.print("th data to memory: ");
        float write_buffer = data_channel[i];
        uint32_t write_buffer_size = sizeof(write_buffer);
        Serial.println(flash.write("log_"+i, property_name, LFLASH_RAW_DATA, (const uint8_t *)&write_buffer, write_buffer_size));
    }
}

void write_data_time(long* data_time)
{
    for(int i = 0; i < DATA_LENGTH; i++)
    {
        long write_buffer = data_time[i];
        uint32_t write_buffer_size = sizeof(write_buffer);
        flash.write("log_"+i, "time", LFLASH_RAW_DATA, (const uint8_t *)&write_buffer, write_buffer_size);
    }
}
