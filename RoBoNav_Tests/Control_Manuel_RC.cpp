#include "pin_definition.h"
#include "RC_acquisition.h"
#include "Wifi_acquisition.h"
#include "ESC_control.h"
#include "delay_nb.h"

bool activerWifi = false; // Activer ou désactiver le wifi

//=======================================================================
//                    Power on setup
//=======================================================================
void setup()
{
  Serial.begin(115200);
  calibrate_RC();
  if (activerWifi)
    Wifi_Init();
  calibrate_ESC();
}

//=======================================================================
//                    Main Program Loop
//=======================================================================
void loop()
{
  if (activerWifi)
    radioWiFi_Decode();
  if (activerWifi)
  {
    throttle = map(Wifi_value[1], RC_value_min, RC_value_max, ESC_NEUTRAL, ESC_MAX);
    rotation = map(Wifi_value[4], RC_value_min, RC_value_max, -1 * rotation_sensitivity, rotation_sensitivity); // Négatif full gauche et positif full droite
  }
  if (!activerWifi)
  {
    throttle = map(RC_value[1], RC_value_min, RC_value_max, ESC_NEUTRAL, ESC_MAX);
    rotation = map(RC_value[4], RC_value_min, RC_value_max, -1 * rotation_sensitivity, rotation_sensitivity); // Négatif full gauche et positif full droite
  }
  consigne_m1 = throttle + rotation + correc_rotation;
  consigne_m2 = throttle - rotation - correc_rotation;

  post_consigne_m1 = m1_control(consigne_m1);
  post_consigne_m2 = m2_control(consigne_m2);

  if (temps_ecoule(200) && true)
  { // Affichage des valeurs sur le terminal série
    if (!activerWifi)
    { // Voies 1 - 8 RC
      for (int i = 1; i <= 8; i++)
      {
        Serial.println("RC" + String(i) + " : " + String(RC_value[i]));
      }
      Serial.println();
    }
    if (activerWifi)
    { // Voies 1 - 8 Wifi
      for (int i = 1; i <= 8; i++)
      {
        Serial.println("Wifi " + String(i) + " : " + String(Wifi_value[i]));
      }
      Serial.println();
    }
    if (true)
    { // Throttle et rotation
      Serial.println("Throttle : " + String(throttle));
      Serial.println("Rotation : " + String(rotation) + "\n");
    }
    if (true)
    { // Consignes pré commande
      Serial.println("Consigne moteur G1 : " + String(consigne_m1));
      Serial.println("Consigne moteur D2 : " + String(consigne_m2) + "\n");
    }
    if (false)
    { // Consignes moteur réelle
      Serial.println("Consigne réelle G1 : " + String(post_consigne_m1));
      Serial.println("Consigne réelle D2 : " + String(post_consigne_m2) + "\n");
    }
  }
}