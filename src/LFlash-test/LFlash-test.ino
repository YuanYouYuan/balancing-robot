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
    Serial.println("write");
    Serial.println(flash.write("section", "property", LFLASH_RAW_DATA,(const uint8_t * )&buffer, size));


    float read_buffer;
    uint32_t read_buffer_size = sizeof(read_buffer);

    Serial.println("read");
    Serial.println(flash.read("section", "property", (uint8_t *)&read_buffer, &read_buffer_size));
    Serial.println(read_buffer);
    Serial.println("wtf");

    uint32_t* a;
    int* b;

    Serial.print("uint32_t*: ");
    Serial.println(sizeof(a));
    Serial.print("int*: ");
    Serial.println(sizeof(b));

}

void loop()
{

}
