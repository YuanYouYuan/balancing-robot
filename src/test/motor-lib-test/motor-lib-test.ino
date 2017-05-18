#include "Motor.h"

Motor motor;
int power = 0;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    if(Serial.available())
    {
        int cmd = Serial.read();
        if(cmd == 'e')
        {
            motor.enable();
            motor.print_info();
        }
        else if(cmd == 'r')
        {
            motor.disable();
            motor.print_info();
        }
        else if(cmd == 's')
        {
            motor.stop();
            motor.print_info();
        }
        else if(cmd == 'q')
        {
            power += 5;
            motor.move(power);
            motor.print_info();
        }
        else if(cmd == 'w')
        {
            power -= 5;
            motor.move(power);
            motor.print_info();
        }
        else 
            Serial.println("e: enable, r: disable, s: stop, q: speed up, w: speed down");
    }
}
