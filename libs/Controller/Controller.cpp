#include "Controller.h"
#include "Arduino.h"
#include "LFlash.h"

Controller::Controller()
{
    flash.begin();
}

void Controller::load_PID_factor()
{
    Serial.print("Loading PID factor from flash...");
    LFlashStatus result[4];
    result[0] = flash.read("factor", "kp", (uint8_t *)&kp, &buf_size);
    result[1] = flash.read("factor", "ki", (uint8_t *)&ki, &buf_size);
    result[2] = flash.read("factor", "kd", (uint8_t *)&kd, &buf_size);
    result[3] = flash.read("factor", "kz", (uint8_t *)&kz, &buf_size);
    bool succeed = true;
    for(int i = 0; i < 4; i++)
        succeed = succeed && (result[i] == LFLASH_OK);
    Serial.println(succeed ? "done" : "failed");
}

void Controller::write_PID_factor()
{
    Serial.print("Writing PID factor into flash...");
    LFlashStatus result[4];
    result[0] = flash.write("factor", "kp", LFLASH_RAW_DATA,(const uint8_t *)&kp, buf_size);
    result[1] = flash.write("factor", "ki", LFLASH_RAW_DATA,(const uint8_t *)&ki, buf_size);
    result[2] = flash.write("factor", "kd", LFLASH_RAW_DATA,(const uint8_t *)&kd, buf_size);
    result[3] = flash.write("factor", "kz", LFLASH_RAW_DATA,(const uint8_t *)&kz, buf_size);
    bool succeed = true;
    for(int i = 0; i < 4; i++)
        succeed = succeed && (result[i] == LFLASH_OK);
    Serial.println(succeed ? "done" : "failed");
}

void Controller::tune_PID_factor()
{
    if(Serial.available())
    {
        switch(Serial.read())
        {
            case 'q':
                kp += KP_ADJUSTMENT;
                break;
            case 'Q':
                kp -= KP_ADJUSTMENT;
                break;
            case 'w':
                ki += KI_ADJUSTMENT;
                break;
            case 'W':
                ki -= KI_ADJUSTMENT;
                break;
            case 'e':
                kd += KD_ADJUSTMENT;
                break;
            case 'E':
                kd -= KD_ADJUSTMENT;
                break;
            case 't':
                kz += KZ_ADJUSTMENT;
                break;
            case 'T':
                kz -= KZ_ADJUSTMENT;
                break;

            // small adjustment
            case 'a': 
                kp += KP_ADJUSTMENT * 0.1;
                break;
            case 'A':
                kp -= KP_ADJUSTMENT * 0.1;
                break;
            case 's':
                ki += KI_ADJUSTMENT * 0.1;
                break;
            case 'S':
                ki -= KI_ADJUSTMENT * 0.1;
                break;
            case 'd':
                kd += KD_ADJUSTMENT * 0.1;
                break;
            case 'D':
                kd -= KD_ADJUSTMENT * 0.1;
                break;
            case 'g':
                kz += KZ_ADJUSTMENT * 0.1;
                break;
            case 'G':
                kz -= KZ_ADJUSTMENT * 0.1;
                break;
            case 'y':
                kp = 0.0;
                ki = 0.0;
                kd = 0.0;
                kz = 0.0;
                break;
            case 'x':
                write_PID_factor();
                break;
            case 'r':
                load_PID_factor();
                break;
            default:
                Serial.println("kp: q/a, ki: w/s, kd: e/d, kz: t/g, capital would shift direction");
                Serial.println("load factor: r, write factor: x, reset factor: f");
                break;
        }
        print_PID_factor();
    }
}

void Controller::print_PID_factor()
{
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(", ki: ");
    Serial.print(ki);
    Serial.print(", kd: ");
    Serial.print(kd);
    Serial.print(", kz: ");
    Serial.print(kz);
    Serial.println();
}

float Controller::get_PID_feedback(float state[3])
{
    p_term = kp * state[0];
    i_term = ki * state[1];
    d_term = kd * state[2];
    feedback = PID_gain * (p_term + i_term + d_term);   
    if(abs(feedback) > max_feedback)
        feedback = (feedback >= 0) ? max_feedback : -1.0*max_feedback;
    return feedback;
}

void Controller::print_PID_feedback()
{
    Serial.print("Feedback: ");
    Serial.print(feedback, 4);
    Serial.print(", p: ");
    Serial.print(p_term, 4);
    Serial.print(", i: ");
    Serial.print(i_term, 4);
    Serial.print(", d: ");
    Serial.print(d_term, 4);
    Serial.println();
}
