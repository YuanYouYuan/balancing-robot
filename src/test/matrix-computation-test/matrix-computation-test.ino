#include <math.h>

#define N1 100
#define N2 200
#define N3 1

float input[3] = {0.1, 0.2, 0.3};
float k = 5;


float w1[3][N1];
float w2[N1][N2];
float w3[N2][N3];

float b1[N1];
float b2[N2];
float b3[N3];

unsigned long timer = 0;

void setup()
{
    Serial.begin(115200);
    randomSeed(analogRead(0));
    generate_random_bias();
    generate_random_weights();
}

void loop()
{
    timer = millis();


    float y1[N1];
    for(int i = 0; i < N1; i++)
        y1[i] = b1[i];
    for(int i = 0; i < N1; i++)
        for(int j = 0; j < 3; j++)
            y1[i] += input[j] * w1[j][i];
    for(int i = 0; i < N1; i++)
        y1[i] = ReLU(y1[i]);

    
    float y2[N2];
    for(int i = 0; i < N2; i++)
        y2[i] = b2[i];
    for(int i = 0; i < N2; i++)
        for(int j = 0; j < N1; j++)
            y2[i] += y1[j] * w2[j][i];
    for(int i = 0; i < N2; i++)
        y2[i] = ReLU(y2[i]);

    float y3[1];
    for(int i = 0; i < N3; i++)
        y3[i] = b3[i];
    for(int i = 0; i < N3; i++)
        for(int j = 0; j < N2; j++)
            y3[i] += y2[j] * w2[j][i];
    for(int i = 0; i < N3; i++)
        y3[i] = k*tanh(y3[i]);

    Serial.println(millis() - timer);



    


}

void generate_random_weights()
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < N1; j++)
            w1[i][j] = (float)random(100)/100.0;

    for(int i = 0; i < N1; i++)
        for(int j = 0; j < N2; j++)
            w2[i][j] = (float)random(100)/100.0;

    for(int i = 0; i < N2; i++)
        for(int j = 0; j < 1; j++)
            w3[i][j] = (float)random(100)/100.0;
}

void generate_random_bias()
{
    for(int i = 0; i < N1; i++)
        b1[i] = (float)random(100)/100.0;
    for(int i = 0; i < N2; i++)
        b2[i] = (float)random(100)/100.0;
    for(int i = 0; i < 1; i++)
        b3[i] = (float)random(100)/100.0;
}



float ReLU(float x)
{
    if(x < 0)
        return 0;
    else
        return x;
}

