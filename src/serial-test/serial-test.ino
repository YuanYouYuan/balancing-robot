const int chan_num = 3;
int data[chan_num];
long time;
void setup()
{
    Serial.begin(115200);
}

void loop()
{

    for(int i = 0; i < chan_num; i++) 
    {
        data[i] = random(1, 80);
        Serial.print(data[i]);
        Serial.print(", ");
    }
    Serial.print(millis()/1000.0);
    Serial.println();

    if(Serial.available())
    {
        char cmd = Serial.read();
        Serial.println(cmd);
    }
    delay(50);
}

