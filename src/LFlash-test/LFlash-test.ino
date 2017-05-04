#include "LFlash.h"

LFlashClass flash;

uint32_t size;
const char * section_name;
const char * property_name;

void setup()
{
    Serial.begin(115200);

    flash.begin();
    float buffer = 123.0;
    size = sizeof(buffer);
    Serial.println("write into memory");
    Serial.println(flash.write("section", "property", LFLASH_RAW_DATA,(const uint8_t * )&buffer, size));


    float read_buffer;
    uint32_t read_buffer_size = sizeof(read_buffer);

    Serial.println("read from memory");
    Serial.println(flash.read("section", "property", (uint8_t *)&read_buffer, &read_buffer_size));
    Serial.println(read_buffer);

}

void loop()
{

}
