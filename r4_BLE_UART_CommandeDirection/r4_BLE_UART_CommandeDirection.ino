/* 
  ******************************
  JF 
  jean.fruitet@free.fr
  ******************************

  Commandes de direction envoyées par BlueTooth à une carte UNO R4 WiFi BlueTooth avec processeur RENESA et module ESP32 d'Expressif

  if (s=="G" || s=="g" || s=="GAUCHE" || s=="gauche" || s=="L" || s=="l" || s=="LEFT" || s=="left" || s=="4" || s=="01")// Hexa
  -> flèche gauche
  
  (s=="D" || s=="d" || s=="DROITE" || s=="droite" || s=="R" || s=="r" || s=="RIGHT" || s=="right" || s=="6" || s=="02") // Hexa   
  -> flèche droite
  
  (s=="H" || s=="h" || s=="HAUT" || s=="haut" || s=="U" || s=="u" || s=="UP" || s=="up" || s=="2" || s=="03") // Hexa   
  -> flèche haut
  
  (s=="B" || s=="b" || s=="BAS" || s=="bas" || s=="DOWN" || s=="down" || s=="8" || s=="04") // Hexa   
  -> flèche bas
    
  sinon
  -> STOP question mark;


Récupération d'une partie des codes sources r4_BLE_ButtonLED.ino et r4_FlechesDirection.ino
JF 08/2024

Avec smartphone, activer le Bluetooth, appairer "UART BLE"
Lancer l'appli "Serial Bluetooth Terminal (Kai Morich)" installée sur le  smartphone
Voir configuration plus bas.
Dans la rubrique du menu sélectionner "Devices", "Bluetooth LE", "UART BLE"
Sélectionner "Bond" devices, sélectionner "UnoR4 BLE Car", connect
Sélectionner "Digital Output"
Written values
Format "Unsigned Littel-endian data)"
Sélectionner le pavé 
            2 : Up
    Left: 4   6: Rigth
            8 : Down 
Toute autre valeur : ?
----------------------------------

  Bluetooth controlled car (that's the eventual goal here)

  My code is shared under the MIT license.
    In a nutshell, use it for anything but you take full responsibility.

Starting with the built-in ArduinoBLE example "Peripheral-ButtonBLE"
Also see:
https://docs.arduino.cc/tutorials/nano-33-ble/bluetooth

  This example creates a Bluetooth® Low Energy peripheral with service that contains a
  characteristic to control an LED

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  See: https://www.arduino.cc/reference/en/libraries/arduinoble/

  Random UUID Generator: https://www.uuidgenerator.net/version4
  example: ea943a1a-2206-4235-970f-ad8127fff9bb

  Characteristics can have a random/custom UUID, or they can use a pre-defined value from the BlueTooth Assigned Numbers list:
  https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf

Examples:
// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID (see assigned numbers document)
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

According to the BLE Assigned Numbers document:
  joystick is 0x03C3
  boolean is 0x2AE2




  ******************************
  Nico78 - French Forum Arduino
  *********************************************************************************************
  Création d'un service BLE pour ajouter un UART BLE (Equivalent du SPP bluetooth Classic)
  *********************************************************************************************
  Exemple basé sur les informations du site web d'Adafruit, de Neil Kolban et de ThingEngineer:
  https://learn.adafruit.com/introducing-adafruit-ble-bluetooth-low-energy-friend/uart-service
  https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/BLE_client/BLE_client.ino
  https://github.com/ThingEngineer/ESP32_BLE_client_uart
  *********************************************************************************************************************************
  Test effectué sur un Arduino nano IOT 33 avec smartphone Android et l'application Serial Bluetooth Terminal (Kai Morich)
  Paramètres de l'application pour le test:
  Terminal Settings: Charset UTF8 (par défaut)
  Display mode: Text (par défaut)
  Receive Settings: CR ou LF (Ne pas mettre les deux), le Moniteur série d'arduino devra être configuré de la même façon
  Send Settings: Character delay -> 0 ms dans ce cas, envoie des données par bloc de 20 caractères au maximum
  ou Send Settings: Character delay -> 1 ms envoie des données au fil de l'eau
  la longueur max de la chaine est définie par la variable tamponCompletedText initialisé a 64 octets que vous pouvez modifier
  *********************************************************************************************************************************
    Infos utiles en FR: https://blog.groupe-sii.com/le-ble-bluetooth-low-energy/
  *********************************************************************************************************************************
*/


#include <ArduinoBLE.h>
// To use ArduinoGraphics APIs, please include BEFORE Arduino_LED_Matrix
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include "flechesdirection2.h"

// Déclaration du service offert et des caractéristiques associées
// Comprendre que TX et RX sont des caractéristiques offertes pour que le client puisse écrire et lire les données
// donc TX et RX sont vues coté client, par conséquent l'arduino lira dans TX et écrira dans RX
BLEService UARTService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic TX("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 20);
BLECharacteristic RX("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLERead | BLENotify, 20);

//Ne pas changer (tampon max de 20 octets que l'on peut écrire dans une caractéristique, restriction du bluetooth)
const unsigned int tamponBluetooth = 20;
char receiveBluetooth[tamponBluetooth + 1] = {0,};

// Vous pouvez changer la valeur tamponCompletedText ci dessous
// suivant la chaine de longueur max que vous souhaitez pouvoir recevoir
const unsigned int tamponCompletedText = 64;
char completedText[tamponCompletedText + 1] = {0,}; // + 1 pour le '\0' de fin de chaine

BLEDevice central;

ArduinoLEDMatrix matrix;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // you can also load frames at runtime, without stopping the refresh
  matrix.loadSequence(flechesdirection2);
  matrix.begin();
  pinMode(LED_BUILTIN, OUTPUT); // use the LED as an output

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // Nom lié au service GENERIC ACCESS
  BLE.setDeviceName("UART Service");

  // Nom qui apparait lors d'un scan
  BLE.setLocalName("UART BLE");

  // Déclaration du service que l'on veut offrir
  //BLE.setAdvertisedServiceUuid(UARTService.uuid());
  BLE.setAdvertisedService(UARTService);

  UARTService.addCharacteristic(TX);
  UARTService.addCharacteristic(RX);

  BLE.addService(UARTService);

  // Démarrer la publication du service
  BLE.advertise();

  Serial.println("UART Service");
}

void loop() {
  int ret = 0;
  static int etat = 0;
  char receptionPortSerie[tamponBluetooth + 1] = {0,};

  // Nous sommes un périphérique esclave (serveur)
  // et nous attendons une connexion centrale maitre (client)

  // En attente de connexion d'un client
  central = BLE.central();

  // etat est utilisé pour éviter la répétition de l'information
  if (etat == 0) {
    etat = 1;
    Serial.println("périphérique esclave (serveur), en attente de connexion d'un client maitre");
    Serial.println("");
  }

  // Test si un appareil s'est connecté
  if (central) {
    Serial.print("Connecté à l'appareil suivant: ");
    Serial.println(central.address());
    Serial.println("");

    digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection

    while (central.connected()) {
      // ***********************************************************************
      // **** Lecture des données reçues du port série *************************
      if (Serial.available() > 0) {
        memset(receptionPortSerie, 0, tamponBluetooth);
        for (unsigned int i = 0; i < 20; ++i) { // lecture par bloc de 20 max
          receptionPortSerie[i] = Serial.read();
          //Serial.println(receptionPortSerie[i]);
          if (receptionPortSerie[i] == '\r' || receptionPortSerie[i] == '\n' ) {
            break;
          }
        }
        // *********************************************************************
        // **** Envoie des données du port Série sur le module Bluetooth **
        // que le marqueur de fin soit reçu ou pas (pas de limite pour l'envoie)
        if (writeBleUART(receptionPortSerie))
        {
          Serial.print("Données envoyées: ");
          Serial.println(receptionPortSerie);
        } else {
          Serial.print("Une erreur s'est produite pour l'envoie des données");
        }
        // *********************************************************************
      }

      //***********************************************************************
      //**** Lecture des données du module Bluetooth **************************
      ret = readBleUART();
      if (ret == 1) {                     // données reçues
        Serial.print("Données reçues sur TX: ");
        Serial.print(receiveBluetooth);
        Serial.print("   Longueur: ");
        Serial.println(strlen(receiveBluetooth));
        Serial.println("");
      } else if (ret == 2) {              // chaine complète (marqueur de fin reçu)
        printSerial(completedText, ret);
      } else if (ret == 0) {              // erreur rencontrée, chaine incomplète
        printSerial(completedText, ret);
      } else {                            // -1  Pas de données!
        //Serial.println("Pas de données! ");
      }
      //*************************************************
      // ******* Votre code personnel ici ***************
      //*************************************************

      if (ret == 2){
        // conversion
        int direction = carDirection((String) completedText);
        Serial.print("Commande reçue ");
        Serial.println(direction);
      
        switch (direction) {
          case 4:           
            matrix.loadFrame(flechesdirection2[leftDir]);
            Serial.print("LEFT ");
            Serial.println("leftDir");
            break;
          case 6:
            matrix.loadFrame(flechesdirection2[rightDir]);
            Serial.print("RIGHT "); 
            Serial.println("rightDir");
            break; 
          case 2:       
            matrix.loadFrame(flechesdirection2[upDir]);
            Serial.print("UP ");
            Serial.println("upDir");
            break;
          case 8:
            matrix.loadFrame(flechesdirection2[downDir]);
            Serial.print("DOWN");
            Serial.println("downDir");
            break;
          default:  // 0 or invalid control
          matrix.loadFrame(flechesdirection2[questionDir]);
            Serial.print("STOP " );
            Serial.println("questionDir");
            break;
        }
      }
    }

    Serial.print("Déconnecté du central: ");
    Serial.println(central.address());
    Serial.println("");
    etat = 0;
    
    digitalWrite(LED_BUILTIN, LOW);         // when the central disconnects, turn off the LED
  }
}

int writeBleUART(char text[]) {
  if (central.connected()) {
    if (RX.writeValue(text, strlen(text)))
      return 1;
  }
  return 0;
}

int readBleUART(void) {
  int etat = -1;
  unsigned int lenData = 0;
  static unsigned int align = 0;

  if (central.connected()) {
    if (TX.written()) {
      lenData = TX.valueLength();
      memset(receiveBluetooth, 0, tamponBluetooth);
      if (TX.readValue(receiveBluetooth, lenData)) {
        etat = 1;
        if (align == 0) {
          memset(completedText, 0, tamponCompletedText + 1);
        }
        if ((align + lenData) < (tamponCompletedText + 1) ) {
          memcpy(completedText + align, receiveBluetooth, lenData);
          align += lenData;
          if (completedText[align - 1] == '\r' || completedText[align - 1] == '\n')
          {
            align = 0;
            etat += 1;
          }
        } else {
          memcpy(completedText + align, receiveBluetooth, tamponCompletedText - align);
          align = 0;
          etat = 0;
        }
      }
    }
  }
  return etat;
}

void printSerial(char text[], int etat) {
  if (etat) {
    Serial.print("Chaine complète: ");
  } else {
    Serial.print("Erreur longueur max de la chaine atteinte: ");
    Serial.print(tamponCompletedText);
    Serial.println(" octets et marqueur de fin non reçu!");
    Serial.println("Rappel, le dernier caractère en byte doit être 13 pour '\\r' ou 10 pour '\\n'");
    Serial.print("*** Chaine incomplète *** : ");
  }
  Serial.print(text);
  Serial.print("   Longueur: ");
  Serial.print(strlen(text));
  Serial.print("   Dernier caractère (byte): ");
  Serial.println((byte)text[strlen(text) - 1]);
  Serial.println("");
}


int carDirection(String s){
  s.trim();
  if (s=="G" || s=="g" || s=="GAUCHE" || s=="gauche" || s=="L" || s=="l" || s=="LEFT" || s=="left" || s=="4" || s=="01")// Hexa
        return 4;
  else if (s=="D" || s=="d" || s=="DROITE" || s=="droite" || s=="R" || s=="r" || s=="RIGHT" || s=="right" || s=="6" || s=="02") // Hexa   
     return 6;
  else if (s=="H" || s=="h" || s=="HAUT" || s=="haut" || s=="U" || s=="u" || s=="UP" || s=="up" || s=="2" || s=="03") // Hexa   
     return 2;
  else if (s=="B" || s=="b" || s=="BAS" || s=="bas" || s=="DOWN" || s=="down" || s=="8" || s=="04") // Hexa   
     return 8;     
  else return 0;
}
