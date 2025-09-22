/***************************************
RoBoNav  2023 - 2024 - GPS_acquisition.h
Version initialement développée pour les GPS Ublox M8N en configuration MNEA
Voir Notes.h pour leur configuration
Cette version de mai 2025 implante la programmation des GPS directement en code binaire UBX 
Voir ../../../GPS/src_icam/DGPS M8N/Base_GPS.ino
****************************************/

#ifndef GPS_acquisition_H
#define GPS_acquisition_H

#include <Wire.h>
#include <HardwareSerial.h>
#include <HMC5883L.h>

#include "pin_definition.h"
#include "utils.h"

//--- COMPASS Definition ----------------------------------------------------//
#define HMC5883L_ADDR 0x1E // utiliser le scanner I2C si pas sûr

//--- GPS Definition --------------------------------------------------------//
const int GPS_RxPin = GPS_Rx;                 // Pins UART GPS communication (see Pin_definition.h)
const int GPS_TxPin = GPS_Tx;                
const int GPS_BAUDRATE = 115200;             // vitesse du GPS

const long baudRates[] = { 9600, 19200, 38400, 57600, 115200, 230400, 460800 };
const int numBauds = sizeof(baudRates) / sizeof(baudRates[0]);

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

const unsigned char UBLOX_REVERT[] PROGMEM = {
  // 0 - Config UBX-CFG-CFG - Revert to default Configuration
  0xB5,0x62,0x06,0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x03,0x1B,0x9A  // REVERT to default configuration
};

const unsigned char UBLOX_CONFIG_PORT[] PROGMEM = {  
  // 0 - Config UBX-PORT - GPS COM-UART1
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x84,0x03,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x7E,0xB8
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xBA,0x4E   // DGPS      UART1 - 115200 - IN:UBX - OUT:UBX+NEMA
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x01,0x00,0x23,0x00,0x00,0x00,0x00,0x00,0xDA,0x0E   // RTK-BASE  UART1 - 115200 - IN:UBX - OUT:UBX+NEMA+RTCM3
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x23,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xDA,0x52   // RTK-ROVER UART1 - 115200 - IN:UBX+NEMA+RTCM3 - OUT:UBX
  //0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x23,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xDC,0x5E   // RTK-ROVER UART1 - 115200 - IN:UBX+NEMA+RTCM3 - OUT:UBX+NMEA
};

const unsigned char UBLOX_MSG_INIT[] PROGMEM = {
  // 1 - Disable NMEA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off

  // 2 - Disable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

  // 3 - Enable UBX
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
  0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

  // 4 - Rate
  //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
  
  // 5 - Configuration permanente
  // Il serait peut-êre possible de sauvegarder la configuration pour éviter de repasser par cette procédure en cas 
  // de déconnection du GPS ?
  // UBX-CFG-CFG (save current config as permanent config)
  // 0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xAB // save current config as permanent config

   // End - Configuration
};

const unsigned char UBX_HEADER[]        = { 0xB5, 0x62 };  // Entête sur deux octetsspécifique des modules U-Blox
const unsigned char NAV_POSLLH_HEADER[] = { 0x01, 0x02 }; // Message POSLLH
const unsigned char NAV_STATUS_HEADER[] = { 0x01, 0x03 }; // Message STATUS
const unsigned char NAV_PVT_HEADER[]    = { 0x01, 0x07 }; // Message PVT

enum _ubxMsgType {  // Les trois messages partagent le même espace en mémoire
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_PVT
};

struct NAV_POSLLH {   // Message élémentaire Voir documentation U-Blox page 
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long height;
  long hMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

struct NAV_STATUS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned char gpsFix;
  char flags;
  char fixStat;
  char flags2;
  unsigned long ttff;
  unsigned long msss;
};

// Ne fonctionne pas avec U-Blox M6N ou M7N.
// Sur U-Blox M8N module veillez au numéro de version du firmware
struct NAV_PVT {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  
  unsigned long iTOW;          // GPS time of week of the navigation epoch (ms)  
  unsigned short year;         // Year (UTC) 
  unsigned char month;         // Month, range 1..12 (UTC)
  unsigned char day;           // Day of month, range 1..31 (UTC)
  unsigned char hour;          // Hour of day, range 0..23 (UTC)
  unsigned char minute;        // Minute of hour, range 0..59 (UTC)
  unsigned char second;        // Seconds of minute, range 0..60 (UTC)
  char valid;                  // Validity Flags (see graphic below)
  unsigned long tAcc;          // Time accuracy estimate (UTC) (ns)
  long nano;                   // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
  unsigned char fixType;       // GNSSfix Type, range 0..5
  char flags;                  // Fix Status Flags
  unsigned char reserved1;     // reserved
  unsigned char numSV;         // Number of satellites used in Nav Solution
  
  long lon;                    // Longitude (deg)
  long lat;                    // Latitude (deg)
  long height;                 // Height above Ellipsoid (mm)
  long hMSL;                   // Height above mean sea level (mm)
  unsigned long hAcc;          // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc;          // Vertical Accuracy Estimate (mm)
  
  long velN;                   // NED north velocity (mm/s)
  long velE;                   // NED east velocity (mm/s)
  long velD;                   // NED down velocity (mm/s)
  long gSpeed;                 // Ground Speed (2-D) (mm/s)
  long heading;                // Heading of motion 2-D (deg)
  unsigned long sAcc;          // Speed Accuracy Estimate
  unsigned long headingAcc;    // Heading Accuracy Estimate
  unsigned short pDOP;         // Position dilution of precision
  short reserved2;             // Reserved
  unsigned long reserved3;     // Reserved
                // New firmware U-Blox M8N
  long headVeh;         // I4 deg Heading of vehicle (2-D), this is only valid when headVehValid is set, otherwise the output is set to the heading of motion
  short magDec;         // I2 1e-2 magDec deg Magnetic declination. Only supported in ADR 4.10 and later.
  unsigned short magAcc;    // U2 1e-2 magAcc deg Magnetic declination accuracy. Only supported in ADR 4.10 and later.
};

union UBXMessage {
  NAV_POSLLH navPosllh;
  NAV_STATUS navStatus;
  NAV_PVT    navPvt;
};



extern HMC5883L compass;

void  init_Compass();
void  calibrate_Compass();
float read_Compass();

void  init_GPS();
bool  update_GPS();
bool  wait_GPSFix( bool storeRTH );

long  detect_gps_baudrate();
int   processGPS();
bool  compareMsgHeader(const unsigned char* msgHeader);
void  calcChecksum(unsigned char* CK, int msgSize);
void  sendUBX( const byte *msg, uint8_t len );
bool  waitForAck(byte msgClass, byte msgID, uint16_t timeout);

float deltaAngle( float source_angle, float target_angle );
float getGPSAngle(double latitudeOrigine, double longitudeOrigne, double latitudeDest, double longitudeDest);
float getGPSDistance(double latitudeOrigine, double longitudeOrigne, double latitudeDest, double longitudeDest);

#endif
