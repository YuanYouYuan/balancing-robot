#include "LFlash.h"

LFlashClass flash;

void setup()
{
    Serial.begin(115200);
    LFlashStatus wtf = flash.begin();
}

void loop()
{

}
