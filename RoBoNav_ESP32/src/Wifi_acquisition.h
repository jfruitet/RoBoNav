#ifndef Wifi_acquisition_H
#define Wifi_acquisition_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "WiFiUDP.h"
#include "RC_acquisition.h"
#include "utils.h"

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