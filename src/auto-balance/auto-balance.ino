//pin4(ENA) weak motor

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Motor.h"
#include "LFlash.h"


MPU6050 IMU;
Motor motor;
LFlashClass flash;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define GYRO_SENSITIVITY 131
#define PIN_LED 7
#define PIN_BTN 6
#define SAMPLE_NUMBER 20
#define SMALL_ANGLE 4
#define DEAD_ANGLE 10
#define DEAD_COUNT 10

#define KP_STEP 0.1
#define KPZ_STEP 0.1
#define KI_STEP 0.005
#define KD_STEP 1

#define SMALL_GAIN 0.5
#define NORMAL_GAIN 1


float angle = 0;
float anglez = 0;
float angle_IMU=0;
float anglez_IMU=0;
float angle_acce = 0;
float angle_offset = 0;
float anglez_offset = 0;
float angle_mean = 0;
float anglez_mean = 0;
float angle_diff = 0;
float angle_summ = 0;
float angle_prev = 0;
float anglez_prev = 0;
float angle_rate = 0;
float anglez_rate = 0;
float angle_list[SAMPLE_NUMBER];
float dt, time, time_pre,time_a;
float powerz=0;
float gy_offset = 0;
float gx_offset = 0;
float gz_offset = 0;


float kp, ki, kd,kpz;
float compl_k = 0.98;

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

    motor.enable();
    motor.set_direction(-1);
    factor_read();
    

    long t = millis();
    while((millis() - t) < 2000)
    {
        get_angle();
        angle_offset += angle;
        anglez_offset += anglez;
    }
    Serial.print("angle offset: ");
    Serial.println(angle_offset);
    
}

void loop() 
{

    
    get_angle();
    tune_PID();
    PID_feedback();
    if(abs(anglez_mean)>20)
    {
      anglez_mean=0;
      anglez_offset=0;
      anglez=0;
    }
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
    Serial.print("kpz: ");
    Serial.print(kpz);
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

    Serial.print("read gy_offset from flash: ");
    Serial.println(gy_offset);
    flash.read("factor", "gy_offset", (uint8_t *)&read_buffer, &read_buffer_size);
    gy_offset = read_buffer;
    
    Serial.print("read gz_offset from flash: ");
    Serial.println(gz_offset);
    flash.read("factor", "gz_offset", (uint8_t *)&read_buffer, &read_buffer_size);
    gz_offset = read_buffer;

    Serial.print("read kpz from flash: ");
    Serial.println(kpz);
    flash.read("factor", "kpz", (uint8_t *)&read_buffer, &read_buffer_size);
    kpz = read_buffer;

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

    write_buffer = kpz;
    Serial.print("write kpz to flash: ");
    Serial.println(kpz);
    flash.write("factor", "kpz", LFLASH_RAW_DATA,(const uint8_t * )&write_buffer, write_buffer_size);

    Serial.println("save done");
}

void safety()
{
    if(abs(angle_mean) >= DEAD_ANGLE)
    {
        motor.stop();
        motor.disable();
        Serial.println("Failed!!!!!!");
        Serial.println("Wait for Re-standup");
        delay(1000);
        angle_summ = 0;
        anglez = 0;
        anglez_offset = 0;
        motor.enable();
        stand_up();
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
        else if(cmd == 't')
            kpz += KPZ_STEP;
        else if(cmd == 'T')
            kpz -= KPZ_STEP;
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
        else if(cmd == 'g')
            kpz += KPZ_STEP*0.1;
        else if(cmd == 'G')
            kpz -= KPZ_STEP*0.1;
        else if(cmd == 'x')
            factor_save();
        else if(cmd == 'r')
           factor_read();

     }
    factor_print();
}

void get_angle()
{
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    time_pre = time;
    time = millis();
    dt = (time - time_pre)/1000.0;
    angle_acce = (180/3.141592) * atan2((float)az, (float)ax) -90;
    angle_rate = (float)(gy - gy_offset)/GYRO_SENSITIVITY;
    anglez_rate = (float)(gz - gz_offset)/GYRO_SENSITIVITY;
    angle_IMU = compl_k * (angle_IMU + angle_rate * dt) + (1 - compl_k) * angle_acce;
    anglez_IMU =anglez_IMU+anglez_rate * dt;
    angle =angle_IMU-angle_offset;
    anglez =anglez_IMU-anglez_offset;
    

#if 0
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print("\n");
#endif

#if 0
    Serial.print(ax);
    Serial.print(", ");
    Serial.print(az);
    Serial.print(", ");
    Serial.print(gy);
    Serial.print(", ");
    Serial.println(gz, 4);
#endif
}

void PID_feedback()
{
    angle_diff = angle - angle_prev;
    angle_mean = (angle + angle_prev)/2.0;
    anglez_mean = (anglez + anglez_prev)/2.0;
    angle_summ = 0.95 * angle_summ + angle;
    angle_prev = angle;
    anglez_prev = anglez;
    if(abs(anglez_mean)>20)
    {
      anglez_mean=0;
      anglez_offset=0;
      anglez=0;
    }
//    if(abs(angle_mean) <= SMALL_ANGLE)
//        float gain = SMALL_GAIN;
//    else
//        float gain = NORMAL_GAIN;

    float p_term = kp * angle_mean;
    float i_term = ki * angle_summ;
    float d_term = kd * angle_diff;
    float p_term_z = kpz * anglez_mean;

    float power = NORMAL_GAIN * (p_term + i_term + d_term);
          powerz = NORMAL_GAIN * p_term_z;

   if (powerz>0) 
   {
      motor.move_two(power,power+powerz);
   }
   else
   {
      motor.move_two(power+abs(powerz),power);
   }
    

#if 1
    Serial.print("prop: ");
    Serial.print(100 * p_term / (p_term + i_term + d_term));
    Serial.print(", ");
    Serial.print(100 * i_term / (p_term + i_term + d_term));
    Serial.print(", ");
    Serial.print(100 * d_term / (p_term + i_term + d_term));
    Serial.print(", ");
#endif

#if 1
    Serial.print("angle_mean: ");
    Serial.print(angle_mean, 2);
    Serial.print(",anglez_mean");
    Serial.print(anglez_mean,2);
    Serial.print(", summ: ");
    Serial.print(angle_summ, 2);
    Serial.print(", diff: ");
    Serial.print(angle_diff, 2);
    Serial.print(", power: ");
    if(abs(power) >= 255)
        Serial.println("MAX");
    else
        Serial.println(power);
#endif
}
void stand_up()
{
      get_angle(); 
      if(angle_mean<0)
        motor.move(150);
      else
        motor.move(-150);
      delay(200);
      angle_prev=angle;
   do{
       get_angle();
       angle_mean=(angle_prev+angle);
       if(angle_mean<0)
         motor.move(-150);
       else
         motor.move(150);             
     }while(abs(angle_mean)>5);        
}

