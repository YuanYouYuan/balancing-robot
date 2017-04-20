void setup()
{
//    Serial1.begin(115200);
//    Serial.begin(115200);
    pinMode(7, OUTPUT);
}

void loop()
{
//    if(Serial1.available())
//        Serial.println(Serial1.read());
    digitalWrite(7, 1);
    delay(500);
    digitalWrite(7, 0);
    delay(500);
}
