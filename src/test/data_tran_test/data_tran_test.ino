#include "DataTransimission.h"

#define NUM_CHAN 4
#define NUM_LIST 1000
int data[NUM_LIST][NUM_CHAN];

DataTransimission DT("12345", "1111111111", IPAddress(192, 168, 0, 101), 5000);

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    DT.begin();
    DT.print_wifi_status();
    DT.connect();

    for(int i = 0; i < NUM_LIST; i++)
        for(int j = 0; j < NUM_CHAN; j++)
            data[i][j] = i * NUM_CHAN + j;
}

void loop()
{
    if(Serial.available())
        if(Serial.read() == 'a')
            Serial.println(DT.send_data((const uint8_t*)data, sizeof(data)));
}


