#include <Arduino.h>
#include <WiFi.h>
#include "AsyncUDP.h"

#define UDP_PORT 1012
AsyncUDP udp;
using namespace std;

const char *ssid = "nom_du_point_d'acces";
const char *password = "mot_de_passe";

IPAddress local_IP(192, 168, 151, 1);
IPAddress gateway(192, 168, 151, 246);
IPAddress subnet(255, 255, 255, 0);

//=======================================================================
//                    Power on setup
//=======================================================================
void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connexion en cours...");
  }
  Serial.println("Connecté au réseau WiFi");
  if (udp.listen(1012))
  {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.gatewayIP());
  }
}

void radioWiFi_Decode()
{
  udp.onPacket([](AsyncUDPPacket packet)
               {
            char* tmpStr = (char*) malloc(packet.length() + 1);
            memcpy(tmpStr, packet.data(), packet.length());
            tmpStr[packet.length()] = '\0'; // ensure null termination
            String mensaje = String(tmpStr);
            free(tmpStr); // Strign(char*) creates a copy so we can delete our one
            Serial.println(mensaje); });
}

void loop()
{
  delay(1000);
  radioWiFi_Decode();
}