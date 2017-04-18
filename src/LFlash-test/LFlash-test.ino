#include "LFlash.h"

LFlashClass flash;

const uint8_t * buffer;
uint32_t size;
const char * section_name;
const char * property_name;

void setup()
{
    Serial.begin(115200);
    flash.begin();
    section_name = "section_name";
    property_name = "property_name";
    char * wtf = "wtf"
    size = sizeof(wtf);
    buffer = wtf;
    Serial.print("size: ");
    Serial.println(size);
    Serial.println(section_name);
    Serial.println(flash.write(section_name, property_name, LFLASH_STRING_DATA, buffer, size));
}

void loop()
{

}
