void setup()
{
    Serial.begin(115200);
    randomSeed(analogRead(0));
}


void loop()
{
    Serial.println(random(300));
    delay(300);
}


