# ARDUINO

JF 08/2024 - jean.fruitet@free.fr

## Docs 
https://docs.arduino.cc/

## Language reference
https://www.arduino.cc/reference/en


## Cartes Arduino UNO 
https://www.locoduino.org/spip.php?article340
https://www.lextronic.fr/arduino-r4-wifi-76819.html

Arduino® UNO-R4 WiFi Bluethooth Matrice de LEDs basée le processeur ARM Cortex-M4 32 bits + module WiFi ESP32 S3 de Expressif
SKU: ABX00087

https://docs.arduino.cc/hardware/uno-r4-wifi/

https://docs.arduino.cc/resources/datasheets/ABX00087-datasheet.pdf

https://docs.arduino.cc/resources/schematics/ABX00087-schematics.pdf

Recherche Google "Arduino R4 Bluetooth example" et Arduino UN R4 exemple"

https://docs.arduino.cc/tutorials/uno-r4-wifi/wifi-examples/


https://srituhobby.com/how-to-use-the-arduino-uno-r4-wifi-board-step-by-step/

## Mise en oeuvre du module Bluetooth

### Outils pour communiqur avec la carte Arduino UNO WiFi BLE

Sur le smartphone Android ou iPad iOS installer l'applet "**Serial Bluetooth Terminal (Kai Morich)**" pour envoyer des chaînes de commandes.

###Code source

https://forum.arduino.cc/t/using-ble-on-the-r4/

**Version n'envoyant qu'un caractère à la fois (peu utile)**
https://community.element14.com/members-area/personalblogs/b/nico-tewinkel-s-blog/posts/arduino-uno-r4-wifi---receiving-commands-via-ble

**Version capable d'envoyer des chaînes de caractères**
https://forum.arduino.cc/t/autour-du-bluetooth-ble/652319


## Mise en oeuvre de la matrice de LEDs
https://community.element14.com/members-area/personalblogs/b/nico-tewinkel-s-blog/posts/arduino-r4-wifi---using-the-led-display

Les exemples sont dans Fichiers/exemples/UNO R4 WiFi/LED_Matrix

##Docs en français
 
https://arduiblog.com/2023/12/31/arduino-uno-r4/

https://www.otronic.nl/fr/schemas-de-raccordement-et-code-dexemple/code-dexemple-arduino-r4-pour-matrice/

http://emery.claude.free.fr/esp32-wifi.html

## Programme de test de la communication BLR série + matrice de LEDs pour afficher la commande


### Mise en oeuvre

Installer l'applet "Serial Bluetooth Terminal (Kai Morich)" sur le  smartphone

Dans l'Arduino IDE importer la carte UNO R4 WiFI et les bibliothèques 

````
#include <ArduinoBLE.h>
// To use ArduinoGraphics APIs, please include BEFORE Arduino_LED_Matrix
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include "flechesdirection2.h"
```


#### Configuration de l'applet "Serial Bluetooth Terminal (Kai Morich)"

Paramètres de l'application pour le test:
  Terminal Settings: Charset UTF8 (par défaut)
  Display mode: Text (par défaut)
  Receive Settings: CR ou LF (Ne pas mettre les deux), le Moniteur série d'arduino devra être configuré de la même façon
  Send Settings: Character delay -> 0 ms dans ce cas, envoie des données par bloc de 20 caractères au maximum
  ou Send Settings: Character delay -> 1 ms envoie des données au fil de l'eau
  la longueur max de la chaine est définie par la variable tamponCompletedText initialisé a 64 octets que vous pouvez modifier


### Editer des matrices de LEDSs 

Avec l'outils ad hoc ou importer le fichier flechesdirections2.h
https://ledmatrix-editor.arduino.cc/

### Importer et compiler depuis ARDUINO IDE le code sources
https://github.com/jfruitet/RoBoNav/r4_BLE_UART_CommandeDirection

Sélectionner la carte UNO R4 WiFI et le COM série proposé pour la connexion

Initialiser la communication série avec le moniteur série de l'Arduino IDE à 115200 

Compiler et téléverser l'exécutable vers le micro controlleur

Allumer le Blutooth sur le smartphone et appairer avec "UART BLE"

Lancer l'applet
Dans la rubrique du menu sélectionner "Devices", "Bluetooth LE", "UART BLE", Connect

Programmer les Macros M1: GAUCHE, M2: DROITE, M3: HAUT, M4: BAS, M5: STOP en mode "send"

#### Traitement des commandes 

Pour plus de souplesse plusieurs commandes sont fusionnées

```
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
```

### Liens
Starting with the built-in ArduinoBLE example "Peripheral-ButtonBLE"
Also see:
https://docs.arduino.cc/tutorials/nano-33-ble/bluetooth

This example creates a Bluetooth® Low Energy peripheral with service that contains a characteristic to control an LED

You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or nRF Connect (Android), to interact with the services and characteristics created in this sketch.

See: https://www.arduino.cc/reference/en/libraries/arduinoble/

Random UUID Generator: https://www.uuidgenerator.net/version4
example: ea943a1a-2206-4235-970f-ad8127fff9bb

Characteristics can have a random/custom UUID, or they can use a pre-defined value from the BlueTooth Assigned Numbers list:
https://btprodspecificationrefs.blob.core.windows.net/assigned-numbers/Assigned%20Number%20Types/Assigned_Numbers.pdf

Examples:

```
// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID (see assigned numbers document)
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
```

According to the BLE Assigned Numbers document:
```
  joystick is 0x03C3
  boolean is 0x2AE2
```

### Nico78 - French Forum Arduino
  
Création d'un service BLE pour ajouter un UART BLE (Equivalent du SPP bluetooth Classic)

Exemple basé sur les informations du site web d'Adafruit, de Neil Kolban et de ThingEngineer:
*  https://learn.adafruit.com/introducing-adafruit-ble-bluetooth-low-energy-friend/uart-service
*  https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/Arduino/BLE_client/BLE_client.ino
*  https://github.com/ThingEngineer/ESP32_BLE_client_uart
*   Infos utiles en FR: https://blog.groupe-sii.com/le-ble-bluetooth-low-energy/
 


