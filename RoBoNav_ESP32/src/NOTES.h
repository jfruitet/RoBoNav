/* Gauthier Ailleret - ICAM Nantes - 2024
      Il est passible d'utiliser les logiciels Mission Planner avec un Pixhawk pour faire transiter les datas du GPS en faisant un passflow
      Pour cela :
         Ouvrir Mission Planner
            une fois ouvert et le GPS Connécté (GPS branché sur le pixhawk et le pixhawk conécté via USB au PC)
            Appuyez sur Connect
            après la connection faire Ctrl+F
            Cliquer sur MAV SERIAL PASS
            Laisser trouner en arrière plan
         Ensuite, sur U-center
            appuyer sur Network connection et se connecter a "tcp://localhost:500" l'icone devient verte
            cliquez sur View, Configuration View
            là, vous pouvez parametrer votre GPS
               -RATE (Rates) 200ms
               -PRT(Ports)  UBX+NMEA in & out
               -NMEA (NMEA Protocol) UART2 & High precision mode
               -CFG (Configuration) save current configuration
   ///  Attention, mission planner/pixhawk réinitialiser les paramètres defini au redémarrage!
   gestion de flottec:\Users\nicolas.ferry\Documents\Arduino\RoboNav\src\ESC_control.cpp
      voir comment envoyer les données à toutes le bouées via l'application (broadcast ?)
   Tester le code si dessous avec un GPS fonctionnel (maintien en position et aller à une position)
   Fonctionnement Boussole.

*/

/*
CALIBRAGE DES ESC
Servo ESC_Calibrage;

  pinMode(PWM_M1, OUTPUT);
  ESC_Calibrage.attach(PWM_M1, ESC_MIN, ESC_MAX);

  switch (RC_value[6]) {
    case 3:
      ESC_Calibrage.writeMicroseconds(ESC_MIN);
      break;
    case 2:
      ESC_Calibrage.writeMicroseconds(ESC_NEUTRAL);
      break;
    case 1:
      ESC_Calibrage.writeMicroseconds(ESC_MAX);
      break;
    default:
      Serial.println("Y'a un pb\n");
      break;
  }


2025/05/02 - JF
A partir de mai 2025 la configuration des GPS UBlox est modifiée pour utiliser le mode binaire UBX plutôt que MNEA.
Voir ../../../GPS


2025/06/05 - JF
Une ESC ayant brûlé, il faut la remplacer.
J'ai fourni deux ESC 40A ZMR reverse. Il faut éventuellement reprogrammer ESC_control.cpp

ATTENTION : Pour éviter les risques de court circuit il ne faut JAMAIS alimenter le PCB du contrôleur de vol ICAM / ARBL 
à la fois avec le câble USB et et l'Ubec des ESC connectées aux batteries  batteries !!!!

*/

/* GPS
La version développée initialement pour les GPS Ublox M8N en configuration MNEA est remplacée à partir de mai 2025 par la programmation
directement en code binaire UBX (Voir ../../../GPS/src_icam/DGPS M8N/Base_GPS.ino)
A partir de septembre 2025 nous introduisons une version GPS RTK Quectel qui remplacera le code GPS_acquisition.cpp
*/

/* Réseau WiFi
L'établissement d'u réseau wiFi pour communique par brodcast avec les bouées à partir d'un smartphone ou d'un PC nécessite
un SSID et un mode passe.
Pour vos tests modifier le fichier RoBoNav_config_WiFi.h 
*/

