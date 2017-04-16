#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//#define DEBUG_ANGLE
//#define DEBUG_MOTOR
//#define DEBUG_RAW_DATA
#define DEBUG_PID


#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

#define MOTOR_A    0
#define MOTOR_B    1
#define MOTOR_STOP 0
#define MOTOR_RUN  1


#define GYRO_SENSITIVITY 131


int motor_number = MOTOR_A;
int motor_action = MOTOR_STOP;
int power = 0;
int step = 5;


MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


#define PIN_LED 7
#define PIN_BTN 6

const int angle_list_number = 5;
const double gy_mean = 235.90;
const double complimentary_const = 0.95;



double gyro_angle = 0;
double acce_angle = 0;
double angle_list[angle_list_number];
double dt, time, time_pre;
double offset = 0;

double angle_mean = 0;
double angle_summ = 0;
double angle_diff = 0;
double angle_offset = 0;

float kp = 7;
float ki = 0;
float kd = 0;
float pid_gain = 1;


void setup() {
    Wire.begin();
    Serial.begin(115200);

    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    //verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(PIN_BTN, INPUT);

    pinMode(PIN_LED, OUTPUT);
    for(int i = 0; i < angle_list_number; i++)
        angle_list[i] = 0;

    time = millis();

    Serial.println("Press button to set angle offset");
    while(digitalRead(PIN_BTN) == 0)
        get_angle();

    for(int i = 0; i < angle_list_number; i++)
        angle_summ += angle_list[i];
    angle_offset = angle_summ / angle_list_number;
    Serial.print("angle offset: ");
    Serial.println(angle_offset);

}

void loop() {

    tune_PID();
    get_angle();
    PID_feedback();
    //if(abs(angle_mean) >= 50)
    //    stop();
}

void tune_PID()
{
    float kp_step = 0.2;
    float ki_step = 0.0;
    float kd_step = 0.2;
    if(Serial.available())
    {
        int a = Serial.read();
        if(a == 'q')
            kp += kp_step;
        else if(a == 'a')
            kp -= kp_step;
        else if(a == 'w')
            ki += ki_step;
        else if(a == 's')
            ki -= ki_step ;
        else if(a == 'e')
            kd += kd_step;
        else if(a == 'd')
            kd -= kd_step;
    }
}

double * get_angle()
{
    double angle;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


    time_pre = time;
	time = millis();
	dt = (time - time_pre)/1000.0;
    acce_angle = 90.0 + (180/3.141592) * atan2((double)az, (double)ax);
	gyro_angle += (double)(gy - gy_mean)/GYRO_SENSITIVITY * dt;

    angle = acce_angle * complimentary_const + gyro_angle * (1 - complimentary_const);
    angle -= angle_offset;


    for(int i = 0; i < angle_list_number -1; i++)
        angle_list[i] = angle_list[i+1];
    angle_list[angle_list_number-1] = angle;

#ifdef DEBUG_RAW_DATA
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print("\n");
#endif

#ifdef DEBUG_ANGLE
    Serial.print("gyro_angle:\t");
    Serial.print(gyro_angle);
    Serial.print("\tacce_angle\t");
    Serial.println(acce_angle);
#endif
    return angle_list;
}

void PID_feedback()
{
    angle_mean = 0;
    angle_summ = 0;
    angle_diff = 0;

    for(int i = 0; i < angle_list_number; i++)
        angle_summ += angle_list[i];
    angle_mean = angle_summ / angle_list_number;

    for(int i = 0; i < angle_list_number-1; i++)
        angle_diff += angle_list[i+1] - angle_list[i];
    angle_diff /= (angle_list_number-1);
    

    if(abs(angle_mean) <= 3)
        pid_gain = -0.5;
    else
        pid_gain = -1.0;
    power = pid_gain * (kp * angle_mean + ki * angle_summ + kd * angle_diff);
    motor(MOTOR_A, power, MOTOR_RUN);
    motor(MOTOR_B, power, MOTOR_RUN);

#ifdef DEBUG_PID
    Serial.print("kp: ");
    Serial.print(kp);
    Serial.print(",ki: ");
    Serial.print(ki);
    Serial.print(",kd: ");
    Serial.print(kd);
    Serial.print("\tangle_mean:\t");
    Serial.print(angle_mean);
    Serial.print("\tangle_summ:\t");
    Serial.print(angle_summ);
    Serial.print("\tangle_diff:\t");
    Serial.print(angle_diff);
    Serial.print("\tpower:\t");
    Serial.println(power);
#endif

}

void stop()
{
    while(true)
    {
        motor(MOTOR_A, 0, MOTOR_STOP);
        motor(MOTOR_B, 0, MOTOR_STOP);
        Serial.println("Failed!!!!!!!");
        delay(1000);
    }
    
}


void motor(int motor_number, int power, int motor_action)
{
    int INX, INY, tmp;
    int PWMX = abs(power);
    int PWMY;

    if(motor_number == MOTOR_A)
        INX = IN1, INY = IN2;
    else if(motor_number == MOTOR_B)
        INX = IN3, INY = IN4;
    else
        Serial.println("Unkown motor number");

    if(power < 0)
        tmp = INX, INX = INY, INY = tmp;

    if(motor_action == MOTOR_STOP)
        PWMY = PWMX;
    else if(motor_action == MOTOR_RUN)
        PWMY = 0;
    else
        Serial.println("Unkown motor action");
        
    analogWrite(INX, PWMX);
    analogWrite(INY, PWMY);

#ifdef DEBUG_MOTOR
    Serial.print("INX: ");
    Serial.print(INX);
    Serial.print("\tINY: ");
    Serial.print(INY);
    Serial.print("\tPWMX: ");
    Serial.print(PWMX);
    Serial.print("\tPWMY: ");
    Serial.println(PWMY);
#endif
}
