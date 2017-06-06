#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
#include "Arduino.h"
#include "LFlash.h"

#define KP_ADJUSTMENT 0.1
#define KI_ADJUSTMENT 0.1
#define KD_ADJUSTMENT 0.1
#define KZ_ADJUSTMENT 0.1

class Controller
{
    public:
        Controller();
        float get_PID_feedback(float state[3]);
        void load_PID_factor();
        void write_PID_factor();
        void tune_PID_factor();
        void print_PID_factor();
        void print_PID_feedback();
        float max_feedback = 255.0;
        float PID_gain     = 1.0;
        float kp           = 0.0;
        float ki           = 0.0;
        float kd           = 0.0;
        float kz           = 0.0;
            
    private:
        LFlashClass flash;
        uint32_t buf_size = sizeof(float);
        float p_term   = 0.0;
        float i_term   = 0.0;
        float d_term   = 0.0;
        float feedback = 0.0;
};
#endif
