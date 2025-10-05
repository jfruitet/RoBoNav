/***************************************
RoBoNav  2023 - 2024 - 2025 - GPS_acquisition.h
Version initialement développée pour les GPS Ublox M8N en configuration MNEA
Voir Notes.h pour leur configuration
La version de mai 2025 implante la programmation des GPS directement en code binaire UBX 
Voir ../../../GPS/src_icam/DGPS M8N/Base_GPS.ino
A partir d'octobre 2025 nous ajoutons le support d'un GPS RTK Quectel LC29HDA
Modifier la ligne de code 21 et 22 pour passer d'un module à l'autre
****************************************/

#ifndef GPS_acquisition_H
#define GPS_acquisition_H

// Version 2025 GPS RTK A tester

#include "GPS_UBX_M8N.h"  // Version UBlox module UBX_M8N
#include "RTK.h"          // Version RTK Quectel module LC29HDA
#include "utils.h"

#define USE_UBX_M8N     false       // Support "UBX_M8N" module
#define USE_RTK_LC29H   true        // Support "RTK_LC29HDA" module

extern RTK_Rover rover;             // Ce code est pour les bouées RTK ; il y a un code similaire pour la base fixe à terre

extern double lat_position_gps;
extern double lng_position_gps;
extern double lat_position_bouee;
extern double lng_position_bouee;
extern double lat_position_RTH;
extern double lng_position_RTH;
extern double lat_position_dest;
extern double lng_position_dest;
extern double lat_position_dest_initiale;
extern double lng_position_dest_initiale;

extern RTK_Rover rover;

extern double lat_position_gps;
extern double lng_position_gps;
extern double lat_position_bouee;
extern double lng_position_bouee;
extern double lat_position_RTH;
extern double lng_position_RTH;
extern double lat_position_dest;
extern double lng_position_dest;
extern double lat_position_dest_initiale;
extern double lng_position_dest_initiale;

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
