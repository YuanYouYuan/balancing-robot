#include <LWiFi.h>

char ssid[] = "12345";
char pass[] = "1111111111";

int status = WL_IDLE_STATUS;

WiFiClass wifi;

void setup()
{
    Serial.begin(115200);
    while(!Serial);
    while(status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID ");
        Serial.println(ssid);
        status = wifi.begin(ssid, pass);
        delay(3000);
    }
    Serial.println("Connected to wifi");
    print_wifi_status();
}

void loop()
{

}

void print_wifi_status()
{
    Serial.print("SSID: ");
    Serial.println(wifi.SSID());
    IPAddress ip = wifi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
    long rssi = wifi.RSSI();
    Serial.print("Signal strength (RSSI): ");
    Serial.print(rssi);
    Serial.println(" dBm");

}

