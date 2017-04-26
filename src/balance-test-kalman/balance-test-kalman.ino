#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Motor.h"
#include "LFlash.h"
#include "Kalman.h"
//#define DEBUG_ANGLE
//#define DEBUG_RAW_DATA
#define DEBUG_PID


MPU6050 IMU;
Motor motor;
LFlashClass flash;

int16_t ax, ay, az;
int16_t gx, gy, gz;
Kalman kalmanX;

#define GYRO_SENSITIVITY 131
#define PIN_LED 7
#define PIN_BTN 6
#define SAMPLE_NUMBER 5
#define SMALL_ANGLE 10
#define DEAD_ANGLE 20
#define MAX_ANGLE 50
#define DEAD_COUNT 10
#define COMPLIMENTARY_FACTOR 0.95

#define KP_STEP 0.1
#define KI_STEP 0.005
#define KD_STEP 1

#define SMALL_GAIN 0.8
#define NORMAL_GAIN 1


float angle = 0;
float angle_gyro = 0;
float angle_acce = 0;
float angle_offset = 0;
float angle_list[SAMPLE_NUMBER];
float dt, time, time_pre;

float gy_offset = 0;


float kp, ki, kd;

float write_buffer;
float read_buffer;
uint32_t write_buffer_size = sizeof(write_buffer);
uint32_t read_buffer_size = sizeof(read_buffer);

int dead_counter = 0;

void setup() 
{
    Serial.begin(115200);
    flash.begin();
    Wire.begin();
    IMU.initialize();
    Serial.println("Testing device connections...");
    Serial.println(IMU.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BTN, INPUT);

    for(int i = 0; i < SAMPLE_NUMBER; i++)
        angle_list[i] = 0;


    time = millis();
    Serial.println("Press button to set gy offset");
    long counter = 0;
    while(!digitalRead(PIN_BTN))
    {
        counter++;
        IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        gy_offset += gy;
    }
    gy_offset /= counter;
    Serial.println("gy_offset: ") ;
    Serial.println(gy_offset, 4) ;

    delay(500);

    angle_offset = get_angle();
    Serial.println("angle_offset: ") ;
    Serial.println(angle_offset, 4) ;

    motor.enable();
    motor.set_direction(-1);

    factor_read();
}

void loop() 
{

    get_angle();
    tune_PID();
    PID_feedback(angle);
    safety();
}


void factor_print()
{
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(",ki: ");
    Serial.print(ki);
    Serial.print(",kd: ");
    Serial.print(kd);
    Serial.print(", ");
}

void factor_read()
{
    Serial.print("read kp from flash: ");
    Serial.println(kp);
    flash.read("factor", "kp", (uint8_t *)&read_buffer, &read_buffer_size);
    kp = read_buffer;

    Serial.print("read ki from flash: ");
    Serial.println(ki);
    flash.read("factor", "ki", (uint8_t *)&read_buffer, &read_buffer_size);
    ki = read_buffer;

    Serial.print("read kd from flash: ");
    Serial.println(kd);
    flash.read("factor", "kd", (uint8_t *)&read_buffer, &read_buffer_size);
    kd = read_buffer;

    Serial.println("read done");
}

void factor_save()
{
    write_buffer = kp;
    Serial.print("write kp to flash: ");
    Serial.println(kp);
    flash.write("factor", "kp", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    write_buffer = ki;
    Serial.print("write ki to flash: ");
    Serial.println(ki);
    flash.write("factor", "ki", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    write_buffer = kd;
    Serial.print("write kd to flash: ");
    Serial.println(kd);
    flash.write("factor", "kd", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    Serial.println("save done");
}

void safety()
{
    if(abs(angle) >= DEAD_ANGLE)
        dead_counter++;
    else
        dead_counter = 0;
    if(dead_counter > DEAD_COUNT)
    {
        motor.stop();
        motor.disable();
        Serial.println("Failed!!!!!!");
        Serial.println("Press button or type z to restart");
        while(!digitalRead(PIN_BTN))
            if(Serial.available())
                if(Serial.read() == 'z')
                    break;
        motor.enable();
        angle_gyro = 0;
    }
}

void tune_PID()
{
    if(Serial.available())
    {
        int cmd = Serial.read();
        if(cmd == 'q')
            kp += KP_STEP;
        else if(cmd == 'Q')
            kp -= KP_STEP;
        else if(cmd == 'w')
            ki += KI_STEP;
        else if(cmd == 'W')
            ki -= KI_STEP ;
        else if(cmd == 'e')
            kd += KD_STEP;
        else if(cmd == 'E')
            kd -= KD_STEP;
        else if(cmd == 'a')
            kp += KP_STEP * 0.1;
        else if(cmd == 'A')
            kp -= KP_STEP * 0.1;
        else if(cmd == 's')
            ki += KI_STEP * 0.1;
        else if(cmd == 'S')
            ki -= KI_STEP * 0.1;
        else if(cmd == 'd')
            kd += KD_STEP * 0.1;
        else if(cmd == 'D')
            kd -= KD_STEP * 0.1;
        else if(cmd == 'x')
            factor_save();
        else if(cmd == 'r')
            factor_read();
    }
    factor_print();
}

float get_angle()
{
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    double gyroYrate=gy/131;
    time_pre = time;
    time = millis();
    dt = (time - time_pre)/1000.0;
    angle_acce = (180/3.141592) * atan2((float)az, (float)ax) -90;
    if(angle_acce >= MAX_ANGLE)
        angle_acce = MAX_ANGLE;
    else if(angle_acce <= -1 *MAX_ANGLE)
        angle_acce = -1 * MAX_ANGLE;
    kalmanX.setAngle(angle_acce);
    double kalAngleX = kalmanX.getAngle(angle_acce, gyroYrate, dt);
    angle=kalAngleX;
  
  
  
#ifdef DEBUG_RAW_DATA
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print("\n");
#endif

#ifdef DEBUG_ANGLE
    Serial.print("angle_gyro_var: ");
    Serial.print(angle_gyro_var, 4);
    Serial.print(", angle_gyro: ");
    Serial.print(angle_gyro, 4);
    Serial.print(", angle_acce: ");
    Serial.print(angle_acce, 4);
    Serial.print(", angle: ");
    Serial.print(angle, 4);
    Serial.print(", dt:\t");
    Serial.println(dt, 4);
#endif

    return angle;
}

void PID_feedback(float angle)
{
    float angle_mean = 0;
    float angle_summ = 0;
    float angle_diff = 0;

    for(int i = 0; i < SAMPLE_NUMBER -1; i++)
        angle_list[i] = angle_list[i+1];
    angle_list[SAMPLE_NUMBER-1] = angle;

    for(int i = 0; i < SAMPLE_NUMBER; i++)
        angle_summ += angle_list[i];
    angle_mean = angle_summ / SAMPLE_NUMBER;

    for(int i = 0; i < SAMPLE_NUMBER-1; i++)
        angle_diff += angle_list[i+1] - angle_list[i];
    angle_diff /= (SAMPLE_NUMBER-1);
    
    if(abs(angle_mean) <= SMALL_ANGLE)
        float gain = SMALL_GAIN;
    else
        float gain = NORMAL_GAIN;
    float power = NORMAL_GAIN * (kp * angle_mean + ki * angle_summ + kd * angle_diff);
    motor.move(power);


#ifdef DEBUG_PID
    Serial.print("angle_mean: ");
    Serial.print(angle_mean, 4);
    Serial.print(", angle_summ: ");
    Serial.print(angle_summ, 4);
    Serial.print(", angle_diff: ");
    Serial.print(angle_diff, 4);
    Serial.print(", power: ");
    Serial.println(power);
#endif

}
