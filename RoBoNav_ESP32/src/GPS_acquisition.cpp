/***************************************
RoBoNav  2023 - 2024 - 2025 GPS_acquisition.cpp
Version développée initialement pour les GPS Ublox M8N en configuration MNEA
Voir Notes.h pour leur configuration
A partir de mai 2025 nous programmons ceux-ci directement en UBX depuis le programme Arduino
Voir ../../../GPS/src_icam/DGPS M8N/Base_GPS.ino
A partir de septembre 2025 nous introduisons une version GPS RTK Quectel qui remplacera ce code
****************************************/

#include "GPS_acquisition.h"

#define NB_GPS_MOY  64

HMC5883L compass;
UBXMessage ubxMessage;

double posLat[NB_GPS_MOY];
double posLng[NB_GPS_MOY];
double filterCoef_GPS[5] = { 0.6, 0.15, 0.1, 0.1, 0.05 };


double lat_position_gps = 0.0;
double lng_position_gps = 0.0;

double lat_position_prev = 0.0;
double lng_position_prev = 0.0;
double lat_position_moy  = 0.0;
double lng_position_moy  = 0.0;



//**************************** BOUSSOLE - COMPASS ***********************************************************************************//
void init_Compass()
{
  slog( 1, "CPS", "init Compass...............", false );

  //--- I2C Init -----------------------------------------//
  //Wire.begin();

  //--- I2C Boussole - Mode Continu ----------------------//
  if( !compass.begin() )
  {
    clog("Error !");
    elog();
    return;
  }

  //--- Set Compass Parameters --------------//
  compass.setRange(HMC5883L_RANGE_1_3GA);             // Set measurement range
  compass.setMeasurementMode(HMC5883L_CONTINOUS);     // Set measurement mode
  compass.setDataRate(HMC5883L_DATARATE_75HZ);        // Set data rate
  compass.setSamples(HMC5883L_SAMPLES_8);             // Set number of samples averaged
  compass.setOffset(0, 0, 0);                         // Set calibration offset

  clog("Done.");
  elog();
}

void calibrate_Compass()
{
    int minX = 0, maxX = 0;
    int minY = 0, maxY = 0;
    int minZ = 0, maxZ = 0;
    int offX = 0, offY = 0, offZ = 0;

    slog( 1, "CPS", "Compass calibration...............", false );
    for( int i=0; i < 100; i++ )
    {
        Vector mag = compass.readRaw();

        // Determine Min / Max values
        if (mag.XAxis < minX) minX = mag.XAxis;
        if (mag.XAxis > maxX) maxX = mag.XAxis;
        if (mag.YAxis < minY) minY = mag.YAxis;
        if (mag.YAxis > maxY) maxY = mag.YAxis;
        if (mag.ZAxis < minZ) minZ = mag.ZAxis;
        if (mag.ZAxis > maxZ) maxZ = mag.ZAxis;

        // Calculate offsets
        offX = (maxX + minX)/2;
        offY = (maxY + minY)/2;
        offZ = (maxZ + minZ)/2;
    }

    compass.setOffset( offX, offY, offZ );                         // Set calibration offset
    clog("Done.");
    elog();
    
    slog( 2, "CPS", "Compass calibration : ", false );
    clog("X="); clog(minX); clog(":"); clog(maxX); clog(" ");
    clog("Y="); clog(minY); clog(":"); clog(maxY); clog(" ");
    clog("Z="); clog(minZ); clog(":"); clog(maxZ); clog(" CAL> ");
    clog(offX); clog(":"); clog(offY); clog(":"); clog(offZ);
    elog();
}

float read_Compass()
{
  //--- Lecture de la boussole -----//
  slog( 3, "CPS", "CAP Boussole - Heading = ", false ); 
  Vector norm = compass.readNormalize();

  // Calculer le cap
  float heading = atan2(norm.YAxis, norm.XAxis) - 1.081041; // correction de 62°37' (Nantes)
  clog( heading ); clog(" Rad - CAP = ");
  
  // Correct for heading < 0deg and heading > 360deg
  while( heading < 0.0 )
    heading += 2 * PI;
  while( heading > 2*PI )
    heading -= 2*PI;

  // Convertir les radians en degrés pour plus de lisibilité.
  float headingDegrees = heading * 180 / PI;

  // Output
  clog( headingDegrees ); clog(" ° "); elog();

  return headingDegrees;
}

/*
void init_Compass()
{
  slog( 1, "CPS", "init Compass...............", false );

  //--- I2C Init -----------------------------------------//
  Wire.begin();
  delay(10);

  //--- I2C Boussole - Mode Continu ----------------------//
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02);     // Registre de mode
  Wire.write(0x00);     // Mode continu
  Wire.endTransmission();
  delay(100);

  clog("Done.");
  elog();
}

void calibrate_Compass()
{
}

float read_Compass()
{
  int16_t x = 0, y = 0, z = 0;
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Envoyer une demande au registre X MSB
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_ADDR, 6); // Demander 6 octets; 2 octets par axe
  delay(1);
  if( Wire.available() == 6 )
  {
    // Read register in this order x,z,y
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
  else
  {
    slog( 2, "CPS", "Erreur de lecture des données du compass.............X", false );
  }

  // Conversion des valeurs brutes en gauss
  float xGauss = x / 1090.0;
  float yGauss = y / 1090.0;
  float zGauss = z / 1090.0;

  // Calculer le cap
  float heading = atan2(-yGauss, xGauss) - 0.523599; // correction de 30°

  // Correction lorsque les signes sont inversés.
  if (heading < 0)
     heading += 2 * PI;

  // Convertir les radians en degrés pour plus de lisibilité.
  float headingDegrees = heading * 180 / PI;
  return headingDegrees;
}

*/

//**************************** GPS ***********************************************************************************//
void init_GPS()
{
  long br = 0;

  slog( 1, "GPS", "init GPS...................", true );
  //--- initialisation UART2 - GPS UART -----------------------//
  // Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RxPin, GPS_TxPin);  OLD TO DEL   !!!!!!!!!!!!!!!!!!
  br = detect_gps_baudrate();
  Serial.println(br);

  //=== Revert to Default Configuration ===
  slog( 2, "GPS", "UBX-GPS Reset and setup PORT confiugration", true);
  sendUBX( UBLOX_REVERT, sizeof(UBLOX_REVERT) );
  detect_gps_baudrate(); 

  //=== Config PORT to 115200 Bauds ===//
  slog( 2, "GPS", "UBX-GPS Config PORT configuration", true);
  sendUBX( UBLOX_CONFIG_PORT, sizeof(UBLOX_CONFIG_PORT) );
  br = detect_gps_baudrate(); 
  // Wait For ACK - 0x06 = CFG, 0x00 = CFG-PRT
  while( br != 115200 ) { 
      slog( 2, "GPS", "ACK not received", true );
      br = detect_gps_baudrate();
      sendUBX( UBLOX_CONFIG_PORT, sizeof(UBLOX_CONFIG_PORT) );
      br = detect_gps_baudrate();
  }
  // Send configuration data in UBX protocol
  slog( 2, "GPS", "UBX-GPS Configuration", true);
  sendUBX( UBLOX_MSG_INIT, sizeof(UBLOX_MSG_INIT) );

  lat_position_prev = 0.0;
  lng_position_prev = 0.0;
  lat_position_moy  = 0.0;
  lng_position_moy  = 0.0;
  
  slog( 2, "GPS", "", true);
  slog( 2, "GPS", "Config GPS Terminated => OK.", true);
}

bool update_GPS()
{
  bool res = false;

  //--- Any new GPS Data from UART ? ---//
  //slog( 1, "GPS", "GPS>", false); clog( Serial2.available() ); elog();
  if( Serial2.available() == 0 )
    return false;

  //--- GPS Read a new command line -----//
  // Search for a NEMA Start command '$'
  int msgType = processGPS();
  if ( msgType == MT_NAV_STATUS ) {
    //Serial.print("[STATUS] gpsFix:");    Serial.print(ubxMessage.navStatus.gpsFix);
    //Serial.println();
  }
  else if ( msgType == MT_NAV_POSLLH ) {
    //lat_position_gps = ubxMessage.navPosllh.lat;
    //lng_position_gps = ubxMessage.navPosllh.lon;
    /*Serial.print("[POSLLH] iTOW:"); Serial.print(ubxMessage.navPosllh.iTOW);
    Serial.print(" lat/lon: "); Serial.print(ubxMessage.navPosllh.lat); Serial.print(","); Serial.print(ubxMessage.navPosllh.lon);
    Serial.print(" hAcc: ");    Serial.print(ubxMessage.navPosllh.hAcc/1000.0f);
    Serial.println();
  */}
  else if ( msgType == MT_NAV_PVT ) {
    lat_position_gps = ubxMessage.navPvt.lat;
    lng_position_gps = ubxMessage.navPvt.lon;
    /*Serial.print("[PVT] SV: "); Serial.print(ubxMessage.navPvt.numSV);
    Serial.print(" fixType: "); Serial.print(ubxMessage.navPvt.fixType);
    Serial.print(" Date:");     Serial.print(ubxMessage.navPvt.year); Serial.print("/"); Serial.print(ubxMessage.navPvt.month); Serial.print("/"); Serial.print(ubxMessage.navPvt.day); Serial.print(" "); Serial.print(ubxMessage.navPvt.hour); Serial.print(":"); Serial.print(ubxMessage.navPvt.minute); Serial.print(":"); Serial.print(ubxMessage.navPvt.second);
    */// Eviter les arrondis qui peuvent faire perdre de la précision
    Serial.print(" lat/lon: "); Serial.print(lat_position_gps); Serial.print(","); Serial.print(lng_position_gps);
    /*Serial.print(" gSpeed: ");  Serial.print(ubxMessage.navPvt.gSpeed/1000.0f);
    Serial.print(" heading: "); Serial.print(ubxMessage.navPvt.heading/100000.0f);
    Serial.print(" hAcc: ");    Serial.print(ubxMessage.navPvt.hAcc/1000.0f);
    Serial.println(); */   

    // Calcul ecart entre la moyenne et la nouvelle position
    float deltaGPS = getGPSDistance( lat_position_moy, lng_position_moy, lat_position_gps, lng_position_gps);
    Serial.print( deltaGPS ); Serial.print(" "); Serial.print( lat_position_moy, 6 ); Serial.print(" "); Serial.println(lng_position_moy, 6 );

    // Si c'est le 1er update => on valide la Lat et Lng PREC
    if( deltaGPS > 1.5 )
    {
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
    else
    {
        // Recalcule de la moyenne avec cette nouvelle valeur (bruit)
        //--- Cumul de la nouvelle position dans la moyenne ---//
        //--- Décalage des éléments de la liste vers la droite de 1 position [0,1,2,3,4] => [x,0,1,2,3] ---//
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

        // Calcul la nouvelle la nouvelle position bouée à partir de la moyenne
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
    res = true;
  }

  return res;
}

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

long detect_gps_baudrate()
{
  long baudrate = 0;
  uint8_t ubxFound = 0;
  uint8_t nmeaFound = 0;
  
  for (int i = 0; i < numBauds; i++) {
    baudrate = baudRates[i];
    Serial2.flush();
    Serial2.end();
    Serial.print("Trying GPS at "); Serial.print(baudrate); Serial.print("... ");
    Serial2.begin(baudrate, SERIAL_8N1, GPS_RxPin, GPS_TxPin);

    bool found = false;
    unsigned long start = millis();
    while( millis() - start < 1000 ) {
      if( Serial2.available() ) {
        char c = Serial2.read();
        
        // NMEA detection: look for '$G'
        if( nmeaFound == 0 && c == '$') {
          ubxFound = 0;
          nmeaFound++;
        } else if( nmeaFound == 1 && c == 'G') {
          nmeaFound++;
          break;
        }
        // UBX detection: look for 0xB5 0x62
        else if( ubxFound == 0 && c == 0xB5) {
          nmeaFound = 0;
          ubxFound++;
        } else if (ubxFound == 1 && c == 0x62) {
          ubxFound++;
          break;
        }
        // Reset Wrong UBX or NMEA Frame
        else {
          nmeaFound = 0;
          ubxFound = 0;
        }
      }
    }

    if( nmeaFound >= 2 || ubxFound >= 2) {
      Serial.println("Found GPS!");
      break;
    } else {
      Serial.println("No data.");
      baudrate = 0;
    }
  }

  return baudrate;
}

bool waitForAck(byte msgClass, byte msgID, uint16_t timeout = 1000)
{
  byte ackPacket[10]; // UBX-ACK-ACK is always 10 bytes
  byte expectedAck[] = {
    0xB5, 0x62,     // Sync chars
    0x05, 0x01,     // Class, ID (ACK-ACK)
    0x02, 0x00,     // Payload length = 2
    msgClass, msgID // Payload: class and ID of original message
    // Followed by checksum
  };

  unsigned long start = millis();
  uint8_t i = 0;

  while (millis() - start < timeout) {
    if (Serial2.available()) {
      byte b = Serial2.read();

      if (i < 10) {
        ackPacket[i++] = b;
      }

      // Once we have the full packet
      if (i == 10) {
        // Compare first 8 bytes
        bool match = true;
        for (uint8_t j = 0; j < 8; j++) {
          if (ackPacket[j] != expectedAck[j]) {
            match = false;
            break;
          }
        }

        // Verify checksum
        if (match) {
          byte ckA = 0, ckB = 0;
          for (int j = 2; j < 8; j++) {
            ckA += ackPacket[j];
            ckB += ckA;
          }
          if (ckA == ackPacket[8] && ckB == ackPacket[9]) {
            return true; // ACK-ACK received
          }
        }

        // Shift buffer to look for next packet if needed
        i = 0;
      }
    }
  }

  return false; // Timeout
}

void sendUBX( const byte *msg, uint8_t len )
{
  bool valid_ack = false;
  
  for(uint8_t i = 0; i < len; i++) {                        
    byte c = pgm_read_byte( msg+i );
    Serial2.write( c );
    Serial.print( c, HEX );        
  }
  Serial2.flush();
  Serial.println();
}

// The last two bytes of the message is a checksum value, used to confirm that the received payload is valid.
// The procedure used to calculate this is given as pseudo-code in the uBlox manual.
void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}


// Compares the first two bytes of the ubxMessage struct with a specific message header.
// Returns true if the two bytes match.
bool compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}


// Reads in bytes from the GPS module and checks to see if a valid message has been constructed.
// Returns the type of the message found if successful, or MT_NONE if no message was found.
// After a successful return the contents of the ubxMessage union will be valid, for the 
// message type that was found. Note that further calls to this function can invalidate the
// message content, so you must use the obtained values before calling this function again.
int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];
  
  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while ( Serial2.available() ) {  
    byte c = Serial2.read();    
    //Serial.write(c);
    
    if ( fpos < 2 ) {
      // For the first two bytes we are simply looking for a match with the UBX header bytes (0xB5,0x62)
      if ( c == UBX_HEADER[fpos] )
        fpos++;
      else
        fpos = 0; // Reset to beginning state.
    }
    else {
      // If we come here then fpos >= 2, which means we have found a match with the UBX_HEADER
      // and we are now reading in the bytes that make up the payload.
      
      // Place the incoming byte into the ubxMessage struct. The position is fpos-2 because
      // the struct does not include the initial two-byte header (UBX_HEADER).
      if ( (fpos-2) < payloadSize )
        ((unsigned char*)(&ubxMessage))[fpos-2] = c;

      fpos++;
      
      if ( fpos == 4 ) {
        // We have just received the second byte of the message type header, 
        // so now we can check to see what kind of message it is.
        if ( compareMsgHeader(NAV_POSLLH_HEADER) ) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH);
        }
        else if ( compareMsgHeader(NAV_STATUS_HEADER) ) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(NAV_STATUS);
        }
        else if ( compareMsgHeader(NAV_PVT_HEADER) ) {
          currentMsgType = MT_NAV_PVT;
          payloadSize = sizeof(NAV_PVT);
        }
        else {
          // unknown message type, bail
          fpos = 0;
          continue;
        }
      }

      if ( fpos == (payloadSize+2) ) {
        // All payload bytes have now been received, so we can calculate the 
        // expected checksum value to compare with the next two incoming bytes.
        calcChecksum(checksum, payloadSize);
      }
      else if ( fpos == (payloadSize+3) ) {
        // First byte after the payload, ie. first byte of the checksum.
        // Does it match the first byte of the checksum we calculated?
        if ( c != checksum[0] ) {
          // Checksum doesn't match, reset to beginning state and try again.
          fpos = 0; 
        }
      }
      else if ( fpos == (payloadSize+4) ) {
        // Second byte after the payload, ie. second byte of the checksum.
        // Does it match the second byte of the checksum we calculated?
        fpos = 0; // We will reset the state regardless of whether the checksum matches.
        if ( c == checksum[1] ) {
          // Checksum matches, we have a valid message.
          return currentMsgType; 
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        // We have now read more bytes than both the expected payload and checksum 
        // together, so something went wrong. Reset to beginning state and try again.
        fpos = 0;
      }
    }
  }
  
  return MT_NONE;
}
