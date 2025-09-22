#include <Wire.h>
#include <TinyGPSPlus.h>

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 230400;
float posLat[5] = {0};
float posLng[5] = {0};
float importance_valeurs_GPS[5] = {0.6, 0.15, 0.1, 0.1, 0.05};
float lat_position_bouee = 0;
float lng_position_bouee = 0;
float lat_dest = 47.27506337196598, lng_dest = -1.5062180946043724;

TinyGPSPlus gps;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("Setup Completed !!");
}

void displayInfo()
{
  Serial.print(F("Location: "));

  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.println();
}

void RemplacerValeurGPS(float valeurlat, float valeurlng)
{
  // Décalage des éléments de la liste vers la droite
  for (int i = 4; i > 0; i--)
  {
    posLat[i] = posLat[i - 1];
    posLng[i] = posLng[i - 1];
  }
  posLat[0] = valeurlat;
  posLng[0] = valeurlng;
}

float getGPSDistance(double latitudeOrigine, double longitudeOrigne, double latitudeDest, double longitudeDest)
{
  // Calcul de la distance entre les deux points GPS
  double lat1Rad = radians(latitudeOrigine);
  double lon1Rad = radians(longitudeOrigne);
  double lat2Rad = radians(latitudeDest);
  double lon2Rad = radians(longitudeDest);
  double dLat = lat2Rad - lat1Rad;
  double dLon = lon2Rad - lon1Rad;
  double a = pow(sin(dLat / 2.0), 2) + cos(lat1Rad) * cos(lat2Rad) * pow(sin(dLon / 2.0), 2);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double distance = c * 6371000.0; // distance en mètres
  return distance;
}

void loop()
{
  lat_position_bouee = 0, lng_position_bouee = 0;
  if(Serial2.available() > 0)
  {
    if (gps.encode(Serial2.read())) 
    {
      
      RemplacerValeurGPS(gps.location.lat(), gps.location.lng());
      /*Serial.println("Lat, Lng : ");
      for (int i = 0; i < 6; i++)
      {
        Serial.print(posLat[i], 6);
        Serial.print(" ");
        Serial.print(posLng[i], 6);
        Serial.println(" ");
      }*/
     displayInfo();
    }
    displayInfo();
  }

  for (int i = 0; i < 5; i++)
  {
    lat_position_bouee += posLat[i] * importance_valeurs_GPS[i];
    lng_position_bouee += posLng[i] * importance_valeurs_GPS[i];
  }
  //Serial.println(getGPSDistance(lat_position_bouee, lng_position_bouee, lat_dest, lng_dest));
  Serial.print(gps.location.lat());
  Serial.print(" ");
  Serial.print(gps.location.lng());
  Serial.println("");
  delay(300);
}