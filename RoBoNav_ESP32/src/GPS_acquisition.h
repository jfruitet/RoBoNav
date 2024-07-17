#ifndef GPS_acquisition_H
#define GPS_acquisition_H

#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <HMC5883L.h>

#include "pin_definition.h"
#include "utils.h"

//--- COMPASS Definition ----------------------------------------------------//
#define HMC5883L_ADDR 0x1E // utiliser le scanner I2C si pas sûr

//--- GPS Definition --------------------------------------------------------//
const int GPS_RxPin = GPS_Rx;                 // Pins UART GPS communication (see Pin_definition.h)
const int GPS_TxPin = GPS_Tx;                
const int GPS_BAUDRATE = 115200;               // vitesse du GPS

extern TinyGPSPlus gps;
extern double lat_position_bouee;
extern double lng_position_bouee;
extern double lat_position_RTH;
extern double lng_position_RTH;
extern double lat_position_dest;
extern double lng_position_dest;
extern double lat_position_dest_initiale;
extern double lng_position_dest_initiale;

extern HMC5883L compass;

void  init_Compass();
void  calibrate_Compass();
float read_Compass();

void  init_GPS();
bool  update_GPS();
bool  wait_GPSFix( bool storeRTH );

float deltaAngle( float source_angle, float target_angle );
float getGPSAngle(double latitudeOrigine, double longitudeOrigne, double latitudeDest, double longitudeDest);
float getGPSDistance(double latitudeOrigine, double longitudeOrigne, double latitudeDest, double longitudeDest);

#endif