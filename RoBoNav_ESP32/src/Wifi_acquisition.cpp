#include "Wifi_acquisition.h"

#define NB_TRY_CONNECTIONS 12
#define PACKET_LENGTH 255

//--- WiFi Access Point Configuration ---//
const char *ssid = "NicoPhone";
const char *password = "12345678";

const int UDP_PORT = 1234;
IPAddress local_IP ( 192, 168, 8, 001 );    // IP de la bouée
IPAddress gateway  ( 255, 255, 255, 255 );    // IP de la gateway (router emettant le WiFi /OU IP Boradcast)
IPAddress subnet   ( 255, 255, 255, 000 );    // Masque de sous-réseau

WiFiUDP udp;

int Wifi_value[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int GPS_value[4]  = {0, 0, 0, 0};

int tryConnect = 0;
char packetBuffer[PACKET_LENGTH];
StaticJsonDocument<PACKET_LENGTH> doc;


void init_WiFi()
{
  slog( 1, "WiFi", "init WiFi...................", true );

  //--- WiFI Mode AP=Access_Point or STA=Station (external router needed) ---//
  WiFi.mode(WIFI_STA);

  //--- Start WiFi Connexion ------------------------------------------------//
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  tryConnect = 0;
  while( (WiFi.status() != WL_CONNECTED) && (tryConnect < NB_TRY_CONNECTIONS) )
  {
    slog( 1, "WiFi", "Connexion en cours...................", true );
    cpu_waitTime(1000);
    tryConnect++;
  }
  
  //--- Met à jour - activerWifi si le Wifi n'est pas présent ceci le désactive ---//
  activerWifi = ( WiFi.status() == WL_CONNECTED );
    
  if( activerWifi )
  {
    slog( 1, "WiFi", "Connecté au réseau WiFi...OK!", true );

    //--- Start UDP Listening Server for UDP packet on specified port --------//
    if( udp.begin(UDP_PORT) )
    {
      slog( 1, "WiFi", "UDP Listening on IP: ", false ); clog( WiFi.localIP().toString().c_str() );
      clog(" / "); clog( WiFi.subnetMask().toString().c_str() );
      clog(" GATEWAY "); clog( WiFi.gatewayIP().toString().c_str() );
      elog();
    }
  }
}

void radioDecode_WiFi()
{
    int packetSize = udp.parsePacket();
    if( packetSize )
    {
        int len = udp.read( packetBuffer, PACKET_LENGTH );
        packetBuffer[len] = 0;
        slog( 2, "WiFi", "Messsage UDP > ", false ); clog( packetBuffer ); elog();
    
        // Parsing JSON packet content
        String message = String( packetBuffer );
        DeserializationError error = deserializeJson(doc, message);
        if( error ) {
            slog( 1, "WiFi", "deserializeJson() failed: ", false ); clog( error.c_str() ); elog();
            return;
        }
      
        int mode = doc["Mode"];
        if (mode == 1) {
          for (int i = 1; i <= 4; i++) {
              Wifi_value[i] = doc["RC" + String(i)];
              Wifi_value[i] = Wifi_value[i] - 1000;
          }
        }
        if (mode == 2) {
          for (int i = 1; i <= 8; i++) {
              Wifi_value[i] = doc["RC" + String(i)];
              Wifi_value[i] = RC_Switch(Wifi_value[i]);
          }
        }
        if (mode == 3) {
          for (int i = 1; i <= 3; i++) {
              GPS_value[i] = doc["GPS"+ String(i)];
              slog( 3, "WiFi", "GPS_Value:", false ); clog( GPS_value[i] ); elog();
          }
        }
    }          
}

void display_WiFi()
{ //--- Display WiFi Telecommande State ----------------------------------------//
  slog( 2, "WiFi", "WiFi Telemetry: ", false ); 
  for (int i = 1; i <= 8; i++) {
       clog("WF"); clog(i); clog(":"); clog( Wifi_value[i] ); clog(" | ");
  }
  elog();
}