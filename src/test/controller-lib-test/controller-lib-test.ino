#include "Controller.h"

Controller c;

void setup()
{
    Serial.begin(115200);
    c.load_PID_factor();
    c.print_PID_factor();
    Serial.println("Getting started");
    delay(2000);
}

void loop()
{
    c.tune_PID_factor();
}


