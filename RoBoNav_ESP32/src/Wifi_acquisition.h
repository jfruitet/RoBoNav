/****************************************
RoBoNav  2023 - 2025 - Wifi_acquisition.h
*****************************************/

#ifndef Wifi_acquisition_H
#define Wifi_acquisition_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "WiFiUDP.h"
#include "RC_acquisition.h"
#include "utils.h"
#include "RoBoNav_config_WiFi.h" // Mot de passe et point d'entrée ssid ; A modifier pouur vos propres tests

extern bool activerWifi;

extern const int UDP_PORT;
extern IPAddress local_IP;
extern IPAddress gateway;
extern IPAddress subnet;

extern WiFiUDP udp;

extern int Wifi_value[9];
extern int GPS_value[4];

void init_WiFi();
void radioDecode_WiFi();
void display_WiFi();

#endif
