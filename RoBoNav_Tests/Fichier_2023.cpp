/*
 * Copyright (c) 2023, Icam - Nantes www.icam.fr
 * All rights reserved.
/* RoboNav Project */
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Servo.h"

const char *ssid = "nom du point d'acces";
const char *password = "mot de passe";

IPAddress local_IP(192, 168, 125, 1); 
IPAddress gateway(192, 168, 125, 148);
IPAddress subnet(255, 255, 255, 0);

static const int RXPin = 3, TXPin = 1;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
// SoftwareSerial ss(RXPin, TXPin);

#define UDP_PORT 1012
WiFiUDP udp;
char packetBuffer[255];

unsigned long RC1_timing = 0;
unsigned long RC2_timing = 0;
unsigned long RC3_timing = 0;
unsigned long RC4_timing = 0;
unsigned long RC5_timing = 0;
unsigned long RC6_timing = 0;
unsigned long RC7_timing = 0;
unsigned long RC8_timing = 0;

unsigned long RC1_ref_timing = 0;
unsigned long RC1_value = 0;
unsigned long RC2_ref_timing = 0;
unsigned long RC2_value = 0;
unsigned long RC3_ref_timing = 0;
unsigned long RC3_value = 0;
unsigned long RC4_ref_timing = 0;
unsigned long RC4_value = 0;

#define ESC_G_pin 17
#define ESC_G_MIN 900
#define ESC_G_MAX 2400
Servo ESC_Helice_G;

#define ESC_D_pin 19
#define ESC_D_MIN 900
#define ESC_D_MAX 2400
Servo ESC_Helice_D;

//=======================================================================
//  RC Radio GPIO Interrupts
//=======================================================================
void IRAM_ATTR ISR_RC1_reading()
{
    unsigned long RC1_duration = 0;
    RC1_timing = micros();
    if (RC1_timing > RC1_ref_timing)
    {
        RC1_duration = RC1_timing - RC1_ref_timing;
        if (RC1_duration < 2500)
            RC1_value = RC1_duration;
    }
    RC1_ref_timing = RC1_timing;
}

void IRAM_ATTR ISR_RC2_reading()
{
    unsigned long RC2_duration = 0;
    RC2_timing = micros();
    if (RC2_timing > RC2_ref_timing)
    {
        RC2_duration = RC2_timing - RC2_ref_timing;
        if (RC2_duration < 2500)
            RC2_value = RC2_duration;
    }
    RC2_ref_timing = RC2_timing;
}

//=======================================================================
//                    Power on setup
//=======================================================================
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Serial1.begin(GPSBaud);
    start_Radio_Wifi();
    // calibrate_RC();
    // calibrate_ESC();
    Serial.println("Setup Completed !!");
}

void start_Radio_Wifi()
{
    // Démarrer le point d'accès WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connexion en cours...");
    }
    Serial.println("Connecté au réseau WiFi");
    WiFi.config(local_IP, gateway, subnet);
    udp.begin(UDP_PORT);
    /*
      WiFi.softAPConfig(local_IP, gateway, subnet);
      WiFi.softAP(ssid, password);
      IPAddress myIP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(myIP);
      udp.begin(UDP_PORT);
      */
}

void calibrate_RC()
{
    pinMode(ESC_G_pin, INPUT);
    attachInterrupt(ESC_G_pin, ISR_RC1_reading, CHANGE);
    pinMode(ESC_D_pin, INPUT);
    attachInterrupt(ESC_D_pin, ISR_RC2_reading, CHANGE);
}

void calibrate_ESC()
{
    ESC_Helice_G.attach(ESC_G_pin, ESC_G_MIN, ESC_G_MAX);
    ESC_Helice_G.write(0);

    Serial.println("ESC Ready Calibration");
    while (Serial.available() <= 0)
        delay(1);
    while (Serial.available() > 0)
        Serial.read();
    // Neutral Point
    Serial.println("Neutral Point");
    ESC_Helice_G.write(80);
    delay(1000);
    while (Serial.available() <= 0)
        delay(1);
    while (Serial.available() > 0)
        Serial.read();
    // Forward Point
    Serial.println("Forward Point");
    ESC_Helice_G.write(160);
    delay(1000);
    while (Serial.available() <= 0)
        delay(1);
    while (Serial.available() > 0)
        Serial.read();
    Serial.println("Backward Point");
    ESC_Helice_G.write(0);
    delay(1000);
    while (Serial.available() <= 0)
        delay(1);
    while (Serial.available() > 0)
        Serial.read();
    Serial.println("Neutral Point");
    ESC_Helice_G.write(80);
    delay(1000);
}

void radioWiFi_Decode()
{
    char *cmd;
    char *token;

    int packetSize = udp.parsePacket();
    if (packetSize)
    {
        udp.read(packetBuffer, packetSize);
        packetBuffer[packetSize] = 0;
        Serial.print(millis());
        Serial.print(' ');
        Serial.println(packetBuffer);

        Serial.print("Paquet reçu de ");
        IPAddress remoteIp = udp.remoteIP();
        Serial.print(remoteIp);
        Serial.print(", port ");
        Serial.println(udp.remotePort());

        /*udp.beginPacket(udp.remoteIP(), udp.remotePort());
        udp.write("Paquet reçu !");
        udp.endPacket();*/

        // Decode UDP Packet
        char *cmd = strtok(packetBuffer, ";");
        if (cmd != NULL)
        {
            // Commande Radio RC1 - 4 Channels
            if (strncmp(cmd, "RC1", 3) == 0)
            {
                token = strtok(NULL, ";");
                RC1_value = atoi(token);
                token = strtok(NULL, ";");
                RC2_value = atoi(token);
                token = strtok(NULL, ";");
                RC3_value = atoi(token);
                token = strtok(NULL, ";");
                RC4_value = atoi(token);

                // Serial.print(RC1_value); Serial.print(" ");
                // Serial.print(RC2_value); Serial.print(" ");
                // Serial.print(RC3_value); Serial.print(" ");
                Serial.print(RC4_value);
                Serial.println("");
            }
        }
    }
}

float getGPSAngle(double latitudeOrigine, double longitudeOrigne, double latitudeDest, double longitudeDest)
{
    // Double precision pour des dérives sub-métriques
    double lat1Rad = radians(latitudeOrigine);
    double lon1Rad = radians(longitudeOrigne);
    double lat2Rad = radians(latitudeDest);
    double lon2Rad = radians(longitudeDest);
    double dLon = lon2Rad - lon1Rad;
    double y = sin(dLon) * cos(lat2Rad);
    double x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
    double angle = fmod(atan2(y, x) * 57.2957795131, 360.0); // DegreeToRadians (atan2(y, x) * 4068.0) / 71.0; // 360/2pi
    if (angle < 0.0)
        angle = angle + 360.0;

    return (float)angle;
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

//=======================================================================
//                    Main Program Loop
//=======================================================================
void loop()
{
    // This sketch displays information every time a new sentence is correctly encoded.
    while (Serial1.available() > 0)
        if (gps.encode(Serial1.read()))
            displayInfo();

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        while (true)
            ;
    }
    return;
    Serial.println(getGPSAngle(47.24347594169088, -1.474032982629585, 47.24369397807607, -1.4737601872387849));
    Serial.println(getGPSDistance(47.24347594169088, -1.474032982629585, 47.24369397807607, -1.4737601872387849));
    Serial.println(getGPSAngle(47.24347594169088, -1.474032982629585, 47.24324239240346, -1.4737011786373964));
    Serial.println(getGPSDistance(47.24347594169088, -1.474032982629585, 47.24324239240346, -1.4737011786373964));
    Serial.println(getGPSAngle(47.24347594169088, -1.474032982629585, 47.24328245257839, -1.4742751713963567));
    Serial.println(getGPSDistance(47.24347594169088, -1.474032982629585, 47.24328245257839, -1.4742751713963567));
    Serial.println(getGPSAngle(47.24347594169088, -1.474032982629585, 47.243664843632715, -1.4743288155794372));
    Serial.println(getGPSDistance(47.24347594169088, -1.474032982629585, 47.243664843632715, -1.4743288155794372));
    Serial.println(fmod(-160, 360));
    Serial.println(fmod(-160, 360) + 360.0);
    radioWiFi_Decode();
    if (RC4_value < 1000)
        RC4_value = 1000;
    if (RC4_value > 2000)
        RC4_value = 2000;
    uint8_t thr = map(RC4_value, 1000, 2000, 0, 160);
    Serial.println(thr);
    ESC_Helice_G.write(thr);
    delay(20);
}
//=======================================================================
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
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
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