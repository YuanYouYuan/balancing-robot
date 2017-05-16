#include <LWiFi.h>

char ssid[] = "12345";
char pass[] = "1111111111";

int status = WL_IDLE_STATUS;

WiFiClient client;
IPAddress server(192,168,0,103);
int port = 5000;

float data[5][1000];

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    while(status != WL_CONNECTED)
    {
        Serial.print("Connecting to wifi ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(3000);
    }
    Serial.println("Connected to wifi");

    Serial.println("Connecting to server ");
    if(client.connect(server, port))
    {
        for(int i = 0; i < 5; i++)
            for(int j; j < 1000; j++)
                data[i][j] = random(1, 100);
        client.write((const uint8_t*)data, sizeof(data));
    }
    else
        Serial.println("connect failed");



}

void loop()
{

}

