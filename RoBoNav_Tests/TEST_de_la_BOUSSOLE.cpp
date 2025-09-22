#include <Wire.h>
#include <Arduino.h>


#define HMC5883L_ADDR 0x1E // Adresse I2C du HMC5883L

void setup()
{
  Serial.begin(115200);
  Wire.begin();  
  // Initialiser le HMC5883L
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); // Registre de mode
  Wire.write(0x00); // Mode continu
  Wire.endTransmission();

  Serial.println("Configuration du compas et du GPS terminée !!");
}

void loop()
{
  // Lire les données du compas
  int16_t x = 0, y = 0, z = 0;
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Envoyer une demande au registre X MSB
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_ADDR, 6); // Demander 6 octets; 2 octets par axe
  if (Wire.available() == 6)
  { 
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
  else
  {
    Serial.println("Erreur de lecture des données du compas");
    return;
  }

  // Conversion des valeurs brutes en gauss
  float xGauss = x / 1090.0;
  float yGauss = y / 1090.0;
  float zGauss = z / 1090.0;

  // Calculer le cap
  float heading = atan2(yGauss, xGauss)+2.70; //correction de 30°

  // Correction lorsque les signes sont inversés.
  if (heading < 0)
    heading += 2 * PI;

  // Convertir les radians en degrés pour plus de lisibilité.
  float headingDegrees = heading * 180 / PI;

  /* Imprimer les valeurs du compas pour le débogage
  Serial.print("Cap : ");
  Serial.println(headingDegrees);

  delay(1000);*/
}
