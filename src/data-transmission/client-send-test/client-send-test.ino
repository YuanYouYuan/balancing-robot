#include <LWiFi.h>

char ssid[] = "12345";
char pass[] = "1111111111";

int status = WL_IDLE_STATUS;

WiFiClient client;
IPAddress server(192,168,0,103);
int port = 5001;

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
    print_wifi_stauts();

    Serial.println("Connecting to server ");
    if(client.connect(server, port))
        Serial.println("Connected");
    else
        Serial.println("connection failed");
}

void loop()
{
    if(Serial.available())
        if(client.connected())
            client.write(Serial.read());
    if(!client.connected())
    {
        Serial.println();
        Serial.println("disconnecting");
        client.stop();
        while(1);
    }

}

void print_wifi_stauts()
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP Adress: ");
    Serial.println(WiFi.localIP());
}
