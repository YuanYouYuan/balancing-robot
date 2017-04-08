// this is simple script to test the encoder of JGA25-371 motor
// there are 334 impulse on the encoder in one cycle

// ENC = encoder, INT = interrupet
#define ENC_PIN_A 2 
#define ENC_PIN_B 4
#define INT_NUM   0 

// encoder data, IMP_NUM = impulse number per revolution
#define GEAR_RATIO 9.28
#define IMP_NUM    334

long motor_rpm = 0;
long encoder_counter = 0;
unsigned long timer = 0;
unsigned long pre_timer = 0;

void setup()
{
    Serial.begin(57600);
    pinMode(ENC_PIN_A, INPUT);
    pinMode(ENC_PIN_B, INPUT);
    attachInterrupt(INT_NUM, encoder_count, FALLING);
}

void loop()
{
    timer = millis();
    if(abs(timer - pre_timer) >= 500)
    {
        detachInterrupt(INT_NUM);
        motor_rpm = (float) encoder_counter * 60 * 2 / (IMP_NUM * GEAR_RATIO);
        encoder_counter = 0;
        pre_timer = millis();
        attachInterrupt(INT_NUM, encoder_count, FALLING);
        Serial.print("rpm: ");
        Serial.println(motor_rpm);
    }
}

void encoder_count()
{
    encoder_counter += 1;
}


