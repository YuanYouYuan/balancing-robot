#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

#define MOTOR_A    0
#define MOTOR_B    1
#define MOTOR_STOP 0
#define MOTOR_RUN  1

int motor_number = MOTOR_A;
int motor_action = MOTOR_STOP;
int power = 0;
int step = 5;

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    Serial.begin(115200);
    Serial.println("Start!!!");
    Serial.println("Q: switch motor, W/E: adjust power, R: stop/run");
}

void loop()
{
    if(Serial.available())
    {
        int cmd = Serial.read();
        if(cmd == 'q')
            motor_number = (motor_number + 1) % 2;
        else if(cmd == 'w')
            power += step;
        else if(cmd == 'e')
            power -= step;
        else if(cmd == 'r')
            motor_action = (motor_action + 1) % 2;
        if(power >= 255)
            power = 255;
        else if(power <= -255)
            power = -255;

        Serial.print("Motor number: ");
        Serial.print(motor_number);
        Serial.print("\tpower: ");
        Serial.print(power);
        Serial.print("\tmotor_action: ");
        Serial.println(motor_action);

        motor(motor_number, power, motor_action);
        Serial.println();
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

    Serial.print("INX: ");
    Serial.print(INX);
    Serial.print("\tINY: ");
    Serial.print(INY);
    Serial.print("\tPWMX: ");
    Serial.print(PWMX);
    Serial.print("\tPWMY: ");
    Serial.println(PWMY);
}
