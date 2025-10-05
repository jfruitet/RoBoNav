/**************************************
RoBoNav - 2025 - RTK_Base.cpp
Développé par Agathe.D, intégré à RoBoNav par N. Ferry - ICAM Nantes
Module GPS RTK Quectel 
***************************************/

#include "RTK.h"


void RTK_Base::begin( bool transparent )
{
  // Init Hardware Serial Communication GPS port
  _gps->begin(115200, SERIAL_8N1, rxPin, txPin);

  resetGPS();
  transparentMode( transparent );       // ESP in transparent Mode relie GPS to Serial pour le debug avec GNSS
  
  // Configurer GPS en mode Base RTK
  configureGPS_BaseRTK( RTK_MODE::MODE_SURVEYIN, 0,0,0 );
}

void RTK_Base::configureGPS_BaseRTK( RTK_MODE mode, double x, double y, double z )
{
  // Base Station - Config Mode
  _RTKmode = mode;
  _gps->println("$PQTMRESTOREPAR*13");            // Restore parametres par défaut
  _gps->println("$PQTMCFGRCVRMODE,W,2*29");       // Configure en mode Base RTK
  _gps->println("$PQTMSAVEPAR*5A");               // Sauvegarde de la configuration
  resetGPS();

  // Survey-in (Mode 3) or Fixed ECEF Base (Mode 2) -- Config Mode
  if( _RTKmode == RTK_MODE::MODE_SURVEYIN )
      _gps->println("$PQTMCFGSVIN,W,1,300,2,0,0,0*23"); // Set the base Station X,Y,Z coordinnates
  else if( _RTKmode == RTK_MODE::MODE_FIXBASE ) {
      //_gps->println("$PQTMCFGSVIN,W,2,0,0,4333751.903,-113958.145,4662601.388*29"); // Set manually the base Station X,Y,Z coordinnates
    initBase_ECEFCoord( x, y, z );
  }
  _gps->println("$PQTMCFGNMEADP,W,3,8,3,3,3,3*39"); // Set decimal precision to maximum for NMEA
  _gps->println("$PQTMSAVEPAR*5A");                 // Sauvegarde de la configuration
  resetGPS();

  // GPS Parameters Config
  _gps->println("$PAIR432,1*22");                 // Output RTCM3 MSM7 (Most detailled messages)
  _gps->println("$PAIR434,1*24");                 // Output RTCM3 antenna position (1005)
  _gps->println("$PAIR436,1*26");                 // Output RTCM3 with satellites ephemerides
  _gps->println("$PAIR062,0,01*0F");              // Enable GCA NMEA Messages
  _gps->println("$PQTMSAVEPAR*5A");               // Sauvegarde de la configuration

  // Message that are not config saved
  if( _RTKmode == RTK_MODE::MODE_SURVEYIN )
    _gps->println("$PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1*58");  // Survey IN status - ON
  else
    _gps->println("$PQTMCFGMSGRATE,W,PQTMSVINSTATUS,0,1*59");  // Survey IN status - OFF
}

void RTK_Base::initBase_ECEFCoord( double x, double y, double z )
{ 
  char nmeaBuffer[100];
  char coreSentence[80];
  
  _RTKmode = RTK_MODE::MODE_FIXBASE;
  snprintf(coreSentence, sizeof(coreSentence), "PQTMCFGSVIN,W,2,0,0,%.4f,%.4f,%.4f", x, y, z);
  
  uint8_t checksum = 0;
  for (int i = 0; coreSentence[i] != '\0'; i++) {
    checksum ^= coreSentence[i];
  }
  
  snprintf(nmeaBuffer, sizeof(nmeaBuffer), "$%s*%02X\r\n", coreSentence, checksum);
  Serial.println(nmeaBuffer);
  _gps->println(nmeaBuffer);
}

void RTK_Base::surveyIn_ECEFCoordinate( String nmea )
{
  // Supression du checksum
  int starIdx = nmea.indexOf('*');
  if( starIdx != -1 )
      nmea = nmea.substring(0, starIdx);

  String fields[13];
  int fieldIndex = 0;
  int startIdx = 0;

  while( startIdx < nmea.length() && fieldIndex < 13 ) {
    int commaIdx = nmea.indexOf(",", startIdx);
    if (commaIdx == -1) {
      fields[fieldIndex++] = nmea.substring(startIdx);
      break;
    } else {
      fields[fieldIndex++] = nmea.substring(startIdx, commaIdx);
      startIdx = commaIdx + 1;
    }
  }
  int ver = fields[1].toInt();
  double tow = fields[2].toDouble();
  int valid = fields[3].toInt();
  int mean_sats = fields[5].toInt();
  int obs = fields[6].toInt();
  int duration = fields[7].toInt();
  double x = fields[8].toDouble();
  double y = fields[9].toDouble(); 
  double z = fields[10].toDouble();
  if( DEBUG_GPS_INFO ){
    Serial.print("X = "); Serial.print(x, 4); Serial.print(", ");
    Serial.print("Y = "); Serial.print(y, 4); Serial.print(", ");
    Serial.print("Z = "); Serial.print(z, 4); Serial.println();
  }
  
  // Le Survey-In est terminé => Ecriture des coord en mode 2 (Write ECEF)
  if( obs == duration ) {
    configureGPS_BaseRTK( RTK_MODE::MODE_FIXBASE, x,y,z );
  }
}
