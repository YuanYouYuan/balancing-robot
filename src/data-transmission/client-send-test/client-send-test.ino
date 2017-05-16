#include <LWiFi.h>

char ssid[] = "BRL03_2.4G";
char pass[] = "biorola33669771";

int status = WL_IDLE_STATUS;

WiFiClient client;
IPAddress server(192,168,0,198);
int port = 5000;
String readline = "wtf";

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




    float angle_list[100];
    float angle_rate_list[100];
    float angle_acce_list[100];
    float power[100];

    data_header = "s";

    client.write((const uint8_t*)angle_list, sizeof(angle_list));
    client.write((const uint8_t*)angle_rate_list, sizeof(angle_list));
    client.write((const uint8_t*)angle_acce_list, sizeof(angle_list));
    client.write((const uint8_t*)power, sizeof(angle_list));

    if(client.connect(server, port))
    {
        client.println(readline);
        client.write((const uint8_t*)data)
        Serial.println("send to server");
        delay(10);
    }
    else
        Serial.println("connect failed");

    while(client.available())
        Serial.write(client.read());
}

void loop()
{
}
