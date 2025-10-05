/**************************************
RoBoNav - 2025 - RTK_Rover.cpp
Développé par Agathe.D, intégré à RoBoNav par N. Ferry - ICAM Nantes
Module GPS RTK Quectel 
***************************************/

#include "RTK.h"


void RTK_Rover::begin( bool transparent )
{
  // Init Hardware Serial Communication GPS port
  _gps->begin(115200, SERIAL_8N1, rxPin, txPin);

  resetGPS();
  transparentMode( transparent );       // ESP in transparent Mode relie le GPS to Serial en boucle infinie pour debugger avec le logiciel du GNSS
  
  // Configurer GPS en mode Rover RTK
  _RTKmode = RTK_MODE::MODE_ROVER;
  _gps->println("$PQTMRESTOREPAR*13");            // Restore parametres par défaut
  _gps->println("$PQTMCFGRCVRMODE,W,1*2A");       // Configure en mode Rover RTK
  _gps->println("$PQTMSAVEPAR*5A");               // Sauvegarde de la configuration
  resetGPS();
  _gps->println("$PAIR062,1,0*3C");               // Turn off GLL messages
  _gps->println("$PAIR062,2,0*3C");               // Turn off GSA messages
  _gps->println("$PAIR062,3,0*3D");               // Turn off GSV messages
  _gps->println("$PAIR062,5,0*3B");               // Turn off VTG messages
  _gps->println("$PAIR062,0,01*0F");              // Enable GCA NMEA Messages
  //_gps->println("$PAIR100,1,0*3A");
  //_gps->println("$PAIR400,1*23");
  _gps->println("$PQTMCFGNMEADP,W,3,8,3,3,3,3*39"); // Set decimal precision to maximum for NMEA
  //_gps->println("$PAIR050,200*21");              // Set ouput interval to 200ms (need compatible gps module)
  _gps->println("$PQTMSAVEPAR*5A");               // Sauvegarde de la configuration
}

void RTK_Rover::inject_RTCM(const uint8_t *data, size_t len) {
  if( parseRTCM3(data, len) )
      _gps->write(data, len); // envoi au module GNSS
}
