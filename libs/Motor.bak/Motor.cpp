#include "Motor.h"
#include "Arduino.h"

Motor::Motor()
{

    pin_EN[0]  = 4;
    pin_IN1[0] = 10;
    pin_IN2[0] = 11;
    pin_EN[1]  = 5;
    pin_IN1[1] = 12;
    pin_IN2[1] = 13;
    int direction = 1;
    for(int i = 0; i < 2; i++)
    {
        pinMode(pin_IN1[i], OUTPUT);
        pinMode(pin_IN2[i], OUTPUT);
        pinMode(pin_EN[i],  OUTPUT);
    }
    disable();
    stop();
}

Motor::Motor(int pin_EN_A, int pin_IN1_A, int pin_IN2_A, int pin_EN_B, int pin_IN1_B, int pin_IN2_B)
{

    pin_EN[0]  = pin_EN_A;
    pin_IN1[0] = pin_IN1_A;
    pin_IN2[0] = pin_IN2_A;
    pin_EN[1]  = pin_EN_B;
    pin_IN1[1] = pin_IN1_B;
    pin_IN2[1] = pin_IN2_B;
    int direction = 1;
    for(int i = 0; i < 2; i++)
    {
        pinMode(pin_IN1[i], OUTPUT);
        pinMode(pin_IN2[i], OUTPUT);
        pinMode(pin_EN[i],  OUTPUT);
    }
    disable();
    stop();
}

void Motor::set_direction(int _direction)
{
    if(_direction >= 0)
        direction = 1;
    else
        direction = -1;
}

void Motor::move(float _power)
{
    int power_value = (int)abs(_power);
    if(power_value >= 255)
        power_value = 255;
    for(int i = 0; i < 2; i++)
    {
        power[i] = _power;
        if((direction * _power) >= 0)
        {
            analogWrite(pin_IN1[i], power_value);
            analogWrite(pin_IN2[i],           0);
        }
        else
        {
            analogWrite(pin_IN1[i],           0);
            analogWrite(pin_IN2[i], power_value);
        }

    }
}

void Motor::move_two(float _power_A, float _power_B)
{
    int power_value_A = (int)abs(_power_A);
    int power_value_B = (int)abs(_power_B);
    if(power_value_A >= 255)
        power_value_A = 255;
    if(power_value_B >= 255)
        power_value_B = 255;
    if((direction * _power_A) >= 0)
    {
        analogWrite(pin_IN1[0], power_value_A);
        analogWrite(pin_IN2[0],             0);
    }
    else
    {
        analogWrite(pin_IN1[0],             0);
        analogWrite(pin_IN2[0], power_value_A);
    }
    if((direction * _power_B) >= 0)
    {
        analogWrite(pin_IN1[1], power_value_B);
        analogWrite(pin_IN2[1],             0);
    }
    else
    {
        analogWrite(pin_IN1[1],           0);
        analogWrite(pin_IN2[1], power_value_B);
    }
 }

void Motor::stop()
{
    for(int i = 0; i < 2; i++)
    {
        power[i] = 0;
        analogWrite(pin_IN1[i], 0);
    }
}

void Motor::enable()
{
    for(int i = 0; i < 2; i++)
    {
        ena[i] = 1;
        digitalWrite(pin_EN[i], 1);
    }
}

void Motor::disable()
{
    for(int i = 0; i < 2; i++)
    {
        ena[i] = 0;
        digitalWrite(pin_EN[i], 0);
    }
}

void Motor::print_info()
{
    Serial.print("Power: ");
    Serial.print(power[0]);
    Serial.print(" / ");
    Serial.print(power[1]);
    Serial.print(", Enable: ");
    Serial.print(ena[0]);
    Serial.print(" / ");
    Serial.print(ena[1]);
    Serial.println();
}
