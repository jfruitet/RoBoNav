/* Gauthier Ailleret - ICAM Nantes
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
La configuration des GPS UBlox est à  modifier pouor passer en mode UBX plutôt que MNEA.
Voir ../../../GPS


2025/06/05 - JF

Une ESC ayant brûlé, il faut la remplacer.
J'ai fourni deux ESC 40A ZMR reverse. Il faut probablement reprogrammer ESC_control.cpp
