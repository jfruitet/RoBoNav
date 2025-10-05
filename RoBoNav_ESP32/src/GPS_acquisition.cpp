/***************************************
RoBoNav  2023 - 2024 - 2025 GPS_acquisition.cpp
Version développée initialement pour les GPS Ublox M8N en configuration MNEA
Voir Notes.h pour leur configuration
A partir de mai 2025 nous programmons ceux-ci directement en UBX depuis le programme Arduino
Voir ../../../GPS/src_icam/DGPS M8N/Base_GPS.ino
A partir de septembre 2025 nous introduisons une version GPS RTK Quectel
****************************************/

#include "GPS_acquisition.h"

#define NB_GPS_MOY  64

// Version 2024 
// HMC5883L compass;
// UBXMessage ubxMessage;

double posLat[NB_GPS_MOY];
double posLng[NB_GPS_MOY];
// Version 2024 
// double filterCoef_GPS[5] = { 0.6, 0.15, 0.1, 0.1, 0.05 }; // Coefficient de Filtrage / N'est plus utilise

// Version 2025 RTK
RTK_Rover rover(Serial2);

double lat_position_gps = 0.0;
double lng_position_gps = 0.0;

double lat_position_prev = 0.0;
double lng_position_prev = 0.0;
double lat_position_moy  = 0.0;
double lng_position_moy  = 0.0;



//**************************** BOUSSOLE - COMPASS ***********************************************************************************//

//**************************** MAPPING HARDWARE - BOUSSOLE-COMPASS ******************************************************************//
void init_Compass()
{
  #if USE_UBX_M8N
    init_Compass_M8N();
  #endif
}

void calibrate_Compass()
{
  #if USE_UBX_M8N
    calibrate_Compass_M8N();
  #endif
}

float read_Compass()
{
  #if USE_UBX_M8N
    return read_Compass_M8N();
  #endif

  return 0.0;
}


//**************************** GPS **************************************************************************************************//

//**************************** MAPPING HARDWARE - GPS ou RTK Module *****************************************************************//
void init_GPS()
{
  lat_position_prev = 0.0;
  lng_position_prev = 0.0;
  lat_position_moy  = 0.0;
  lng_position_moy  = 0.0;
  
  #if USE_UBX_M8N
    init_GPS_M8N();
  #elif USE_RTK_LC29H
    rover.begin( false );
  #endif
}

bool update_GPS()
{
  bool res = false;
  float deltaGPS = 0.0;

  // Update une nouvelle coordonnées
  #if USE_UBX_M8N
    res = update_GPS_M8N();
    if( !res ) return false;

    // mise à jour des GPS coordinates passage par reference avec lat_position_gps ET lng_position_gps
    getGPScoordinates_M8N( lat_position_gps, lng_position_gps );
  #elif USE_RTK_LC29H
    res = rover.update();
    if( !res ) return false;

    // mise à jour des GPS coordinates passage par reference avec lat_position_gps ET lng_position_gps
    rover.getGPScoordinates( lat_position_gps, lng_position_gps );
  #endif
  
  // Calcul ecart entre la moyenne et la nouvelle position
  deltaGPS = getGPSDistance( lat_position_moy, lng_position_moy, lat_position_gps, lng_position_gps);

  // Affichage Log GPS
  Serial.print(" lat/lon: "); Serial.print(lat_position_gps); Serial.print(","); Serial.print(lng_position_gps);
  Serial.print( deltaGPS ); Serial.print(" "); Serial.print( lat_position_moy, 10 ); Serial.print(" "); Serial.println(lng_position_moy, 10 );

  // Calcul de la Coordonnée moyennée
  if( deltaGPS > 1.0 )  // Si distance de l'ecart est significatif
  {   // On Définit le nouveau point comme => la nouvelle moyenne
      // ET on restaure le point de target initial (suppression du cumul du bruit)
      
      // Fixe comme étant la nouvelle position bouée
      lat_position_bouee = lat_position_gps;
      lng_position_bouee = lng_position_gps;
      
      //--- Fixe la nouvelle moyenne à cette position ---//
      for( int i = 0; i < NB_GPS_MOY; i++ )
      {
        posLat[i] = lat_position_bouee;
        posLng[i] = lng_position_bouee;
      }

      //--- Store la nouvelle position comme la moyenne (previous au tour d'après) ---//
      lat_position_moy  = lat_position_bouee;
      lng_position_moy  = lng_position_bouee;
      lat_position_prev = lat_position_bouee;
      lng_position_prev = lng_position_bouee;

      // Restaure la target initiale //
      lat_position_dest = lat_position_dest_initiale;
      lng_position_dest = lng_position_dest_initiale;
  }
  else  // Si c'est des écarts faibles / bruit
  {
      // Recalcule de la moyenne avec cette nouvelle valeur (bruit)
      //--- Cumul de la nouvelle position dans la moyenne ---//
      //--- Décalage des éléments de la liste vers la droite de 1a position [0,1,2,3,4] => [x,0,1,2,3] ---//
      for (int i = NB_GPS_MOY-1; i > 0; i--)
      {
        posLat[i] = posLat[i - 1];
        posLng[i] = posLng[i - 1];
      }
      //--- Store la nouvelle valeur ---//
      posLat[0] = lat_position_gps;
      posLng[0] = lng_position_gps;

      //--- Filtering GPS position -----------------------------------------------//
      lat_position_moy = 0.0;
      lng_position_moy = 0.0;
      for (int i = 0; i < NB_GPS_MOY; i++)
      {
        lat_position_moy += posLat[i]; // * filterCoef_GPS[i];
        lng_position_moy += posLng[i]; // * filterCoef_GPS[i];
      }

      // Calcul la nouvelle position bouée à partir de la moyenne
      lat_position_moy = lat_position_moy * (1.0/NB_GPS_MOY);
      lng_position_moy = lng_position_moy * (1.0/NB_GPS_MOY);
      lat_position_bouee = lat_position_gps;
      lng_position_bouee = lng_position_gps;

      double delta_Lat = lat_position_gps - lat_position_prev;
      double delta_Lng = lng_position_gps - lng_position_prev;
      
      // Decale la Target de la même erreur //
      lat_position_dest += delta_Lat;
      lng_position_dest += delta_Lng;

      //--- Store la nouvelle position comme la nouvelle (previous au tour d'après) ---//
      lat_position_prev = lat_position_gps;
      lng_position_prev = lng_position_gps;
  }

  // Nouvel Update Ok
  return res;
}


//----------------------- Independant & Configuration functions -----------------------------------------------------------//

bool wait_GPSFix( bool storeRTH )
{
  bool res = false;
  slog( 1, "GPS", "Wait a GPS Fix......................", true );

  //--- Wait for a GPS Fix ---//
  while( !res )    // OU dépassement d'un temps trop long ?
  {
    res = update_GPS();
    delay(100);
  }

  if( storeRTH ) {
     //--- Store la position de Return to Home ---//
    lat_position_RTH = lat_position_moy;
    lng_position_RTH = lng_position_moy;
    lat_position_dest_initiale = lat_position_RTH;
    lng_position_dest_initiale = lng_position_RTH;
    lat_position_dest = lat_position_dest_initiale;
    lng_position_dest = lng_position_dest_initiale;
    slog( 1, "GPS", "RTH GPS Fix Lat:", false ); clog( lat_position_RTH ); clog( " , Lng:" ); clog( lng_position_RTH ); elog();
  }
  
  return res;
}


float deltaAngle( float source_angle, float target_angle )
{ //--- delta angle - Angle in degress [0 - 360] ---//
  float delta_angle = target_angle - source_angle;
  
  //--- Calibrate angle between 0 and 360 degree ---//
  while( delta_angle < 0.0 )
     delta_angle += 360.0;
  while( delta_angle > 360.0 )
     delta_angle -= 360.0;

  // Final Angle Result is 0-centered and between [-180,0,+180]
  if( delta_angle > 180.0 )
      delta_angle = delta_angle - 360.0;

  return delta_angle;
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


