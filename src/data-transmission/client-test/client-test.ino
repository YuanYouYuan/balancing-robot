#include <LWiFi.h>

char ssid[] = "BRL03_2.4G";
char pass[] = "biorola33669771";

int status = WL_IDLE_STATUS;

WiFiClient client;
IPAddress server(192,168,0,198);
int port = 5000;
String readline;

void setup()
{
    Serial.begin(115200);
    while(!Serial);

    while(status != WL_CONNECTED)
    {
        Serial.print("Connecting to wifi ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(10000);
    }
    Serial.println("Connected to wifi");
    //Serial.println("Connecting to server ");

}

void loop()
{
    if(Serial.available())
    {
        char cmd = Serial.read();
        if(cmd == '\r')
        {
            Serial.print("User input: ");
            Serial.println(readline);
            if(client.connect(server, port))
                client.println(readline);
            readline = "";
        }
        else
            readline += cmd;
    }

    if(client.available())
    {
        Serial.print("Received from server: ");
        Serial.print(client.read());
    }

}
