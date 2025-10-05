/**************************************
RoBoNav - 2025 - RTK.cpp
Développé par Agathe.D, intégré à RoBoNav par N. Ferry - ICAM Nantes
Module GPS RTK Quectel 
***************************************/

#include "RTK.h"

// ======================= CRC24Q TABLE POUR RTCM ========================
static const unsigned crc24q[256] = {
    0x00000000, 0x01864CFB, 0x028AD50D, 0x030C99F6,
    0x0493E6E1, 0x0515AA1A, 0x061933EC, 0x079F7F17,
    0x08A18139, 0x0927CDC2, 0x0A2B5434, 0x0BAD18CF,
    0x0C3267D8, 0x0DB42B23, 0x0EB8B2D5, 0x0F3EFE2E,
    0x10C54E89, 0x11430272, 0x124F9B84, 0x13C9D77F,
    0x1456A868, 0x15D0E493, 0x16DC7D65, 0x175A319E,
    0x1864CFB0, 0x19E2834B, 0x1AEE1ABD, 0x1B685646,
    0x1CF72951, 0x1D7165AA, 0x1E7DFC5C, 0x1FFBB0A7,
    0x200CD1E9, 0x218A9D12, 0x228604E4, 0x2300481F,
    0x249F3708, 0x25197BF3, 0x2615E205, 0x2793AEFE,
    0x28AD50D0, 0x292B1C2B, 0x2A2785DD, 0x2BA1C926,
    0x2C3EB631, 0x2DB8FACA, 0x2EB4633C, 0x2F322FC7,
    0x30C99F60, 0x314FD39B, 0x32434A6D, 0x33C50696,
    0x345A7981, 0x35DC357A, 0x36D0AC8C, 0x3756E077,
    0x38681E59, 0x39EE52A2, 0x3AE2CB54, 0x3B6487AF,
    0x3CFBF8B8, 0x3D7DB443, 0x3E712DB5, 0x3FF7614E,
    0x4019A3D2, 0x419FEF29, 0x429376DF, 0x43153A24,
    0x448A4533, 0x450C09C8, 0x4600903E, 0x4786DCC5,
    0x48B822EB, 0x493E6E10, 0x4A32F7E6, 0x4BB4BB1D,
    0x4C2BC40A, 0x4DAD88F1, 0x4EA11107, 0x4F275DFC,
    0x50DCED5B, 0x515AA1A0, 0x52563856, 0x53D074AD,
    0x544F0BBA, 0x55C94741, 0x56C5DEB7, 0x5743924C,
    0x587D6C62, 0x59FB2099, 0x5AF7B96F, 0x5B71F594,
    0x5CEE8A83, 0x5D68C678, 0x5E645F8E, 0x5FE21375,
    0x6015723B, 0x61933EC0, 0x629FA736, 0x6319EBCD,
    0x648694DA, 0x6500D821, 0x660C41D7, 0x678A0D2C,
    0x68B4F302, 0x6932BFF9, 0x6A3E260F, 0x6BB86AF4,
    0x6C2715E3, 0x6DA15918, 0x6EADC0EE, 0x6F2B8C15,
    0x70D03CB2, 0x71567049, 0x725AE9BF, 0x73DCA544,
    0x7443DA53, 0x75C596A8, 0x76C90F5E, 0x774F43A5,
    0x7871BD8B, 0x79F7F170, 0x7AFB6886, 0x7B7D247D,
    0x7CE25B6A, 0x7D641791, 0x7E688E67, 0x7FEEC29C,
    0x803347A4, 0x81B50B5F, 0x82B992A9, 0x833FDE52,
    0x84A0A145, 0x8526EDBE, 0x862A7448, 0x87AC38B3,
    0x8892C69D, 0x89148A66, 0x8A181390, 0x8B9E5F6B,
    0x8C01207C, 0x8D876C87, 0x8E8BF571, 0x8F0DB98A,
    0x90F6092D, 0x917045D6, 0x927CDC20, 0x93FA90DB,
    0x9465EFCC, 0x95E3A337, 0x96EF3AC1, 0x9769763A,
    0x98578814, 0x99D1C4EF, 0x9ADD5D19, 0x9B5B11E2,
    0x9CC46EF5, 0x9D42220E, 0x9E4EBBF8, 0x9FC8F703,
    0xA03F964D, 0xA1B9DAB6, 0xA2B54340, 0xA3330FBB,
    0xA4AC70AC, 0xA52A3C57, 0xA626A5A1, 0xA7A0E95A,
    0xA89E1774, 0xA9185B8F, 0xAA14C279, 0xAB928E82,
    0xAC0DF195, 0xAD8BBD6E, 0xAE872498, 0xAF016863,
    0xB0FAD8C4, 0xB17C943F, 0xB2700DC9, 0xB3F64132,
    0xB4693E25, 0xB5EF72DE, 0xB6E3EB28, 0xB765A7D3,
    0xB85B59FD, 0xB9DD1506, 0xBAD18CF0, 0xBB57C00B,
    0xBCC8BF1C, 0xBD4EF3E7, 0xBE426A11, 0xBFC426EA,
    0xC02AE476, 0xC1ACA88D, 0xC2A0317B, 0xC3267D80,
    0xC4B90297, 0xC53F4E6C, 0xC633D79A, 0xC7B59B61,
    0xC88B654F, 0xC90D29B4, 0xCA01B042, 0xCB87FCB9,
    0xCC1883AE, 0xCD9ECF55, 0xCE9256A3, 0xCF141A58,
    0xD0EFAAFF, 0xD169E604, 0xD2657FF2, 0xD3E33309,
    0xD47C4C1E, 0xD5FA00E5, 0xD6F69913, 0xD770D5E8,
    0xD84E2BC6, 0xD9C8673D, 0xDAC4FECB, 0xDB42B230,
    0xDCDDCD27, 0xDD5B81DC, 0xDE57182A, 0xDFD154D1,
    0xE026359F, 0xE1A07964, 0xE2ACE092, 0xE32AAC69,
    0xE4B5D37E, 0xE5339F85, 0xE63F0673, 0xE7B94A88,
    0xE887B4A6, 0xE901F85D, 0xEA0D61AB, 0xEB8B2D50,
    0xEC145247, 0xED921EBC, 0xEE9E874A, 0xEF18CBB1,
    0xF0E37B16, 0xF16537ED, 0xF269AE1B, 0xF3EFE2E0,
    0xF4709DF7, 0xF5F6D10C, 0xF6FA48FA, 0xF77C0401,
    0xF842FA2F, 0xF9C4B6D4, 0xFAC82F22, 0xFB4E63D9,
    0xFCD11CCE, 0xFD575035, 0xFE5BC9C3, 0xFFDD8538,
};

uint32_t RTK_Common::computeCRC24Q(const uint8_t *data, size_t length) {
    uint32_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc24q[data[i] ^ (unsigned char)(crc>>16)];
    }
    return (crc & 0x00ffffff);
}

void RTK_Common::resetGPS()
{    // Reboot Module
    _gps->println("$PAIR023*3B");
    delay( 2000 );      // Force wait for module reset complete
}

void RTK_Common::transparentMode( bool mode )
{
  // Transparent Mode - Direct to Serial console
  while( mode ) {
    _RTKmode = RTK_MODE::MODE_PT;

    while( _gps->available() )
      Serial.write( _gps->read() );

    while( Serial.available() )
      _gps->write( Serial.read() );
  }
}

bool RTK_Common::invalidFrame( const char *err_msg, const char *buffer ) {
  if( DEBUG_GPS_INFO )
  { // Debug Print Buffer
    Serial.println( err_msg ); 
    //Serial.println( buffer ); 
  }
  return false;
}

void RTK_Common::getGPScoordinates( double &latitude, double &longitude ) {
  latitude  = _latitude;
  longitude = _longitude;
}

void RTK_Common::frameDebug_Display() {
  char hexa[3] = { 0,0,0 };

  if( DEBUG_GPS_INFO )
  { // Debug Print Buffer
    String debugMesg = "";
    for( uint16_t i = 0; i < _bufferIndex; i++ ) {
      snprintf(hexa, 3, "%02X", _rawBuffer[i]);
      debugMesg += hexa;
      debugMesg += ' ';
    }
    Serial.println( debugMesg );
    Serial.println( (char*)_rawBuffer );
  }
}


void RTK_Common::inject_NMEA(const String& nmea) {
  String framed = addNMEAchecksum(nmea);
  _gps->println(framed);
}

String RTK_Common::addNMEAchecksum(const String& message) {
  uint8_t checksum = 0;
  char checksumStr[4];

  // Calculer et Formater le checksum en deux chiffres hexadécimaux
  for (int i = 1; i < message.length(); i++)
    checksum ^= message[i];
  sprintf(checksumStr, "*%02X", checksum);

  // Construire la trame complète
  String fullSentence = message + String(checksumStr) + "\r\n";
  return fullSentence;
}

bool RTK_Common::checkNMEACRC( const String &nmea ) {
  int asterisk = nmea.indexOf('*');
  if (asterisk == -1 || asterisk + 2 >= nmea.length()) return false;

  uint8_t checksum = 0;
  for (int i = 1; i < asterisk; i++) {
    checksum ^= nmea[i];
  }

  String crcStr = nmea.substring(asterisk + 1, asterisk + 3);
  uint8_t crcFromStr = strtol(crcStr.c_str(), NULL, 16);
  return (checksum == crcFromStr);
}

bool RTK_Common::update()
{
  _validFrame = false;

  /***********************************************************/
  /*** Decodage GPS Frame ***/
  /***********************************************************/
  while( _gps->available() )
  {
    uint8_t byteIn = _gps->read();
    // Empile tant que le buffer n'est pas plein
    if( _bufferIndex < MAX_BUFFER )
        _rawBuffer[_bufferIndex++] = byteIn;    

    // Initialisation de la trame
    if( _bufferIndex == 1 ) {
        _receivingRTCM = ( byteIn == 0xD3 );
        _receivingNMEA = ( byteIn == '$' );
        if( !_receivingRTCM && !_receivingNMEA )
            _bufferIndex = 0;
        continue;
    }
    // Détection de fin de trame
    else if( (_bufferIndex >= MAX_BUFFER) ||
             (_receivingNMEA && _bufferIndex >= 2 && _rawBuffer[_bufferIndex-2] == 0x0D && _rawBuffer[_bufferIndex-1] == 0x0A ) ||
             (_receivingRTCM && _bufferIndex >= 6 && _bufferIndex >= (((_rawBuffer[1] & 0x03) << 8) | _rawBuffer[2]) + 6 )
            )
    {   // Store end of Trame
        _rawBuffer[_bufferIndex] = 0;
        frameDebug_Display();

        // Decode/parse la trame NMEA (pour affichage coord) et RTCM (pour envoi RTK en UDP au Rover)
        if( _receivingNMEA )
            _validFrame = parseNMEA( _rawBuffer, _bufferIndex );
        else if( _receivingRTCM )
            _validFrame = parseRTCM3( _rawBuffer, _bufferIndex );
        else
            _validFrame = invalidFrame( "Invalid niNMEA-niRTCM ", (const char *)_rawBuffer );
        
        // Envoi de la trame RTCM de correction de la base RTK -> vers les Rovers RTK
        if( _RTKmode == RTK_MODE::MODE_FIXBASE && _sendUDP && _receivingRTCM && _validFrame )
            _sendUDP( _rawBuffer, _bufferIndex );

        // Clean Reception Buffer
        _receivingRTCM = false;
        _receivingNMEA = false;
        _bufferIndex = 0;
        break;
    }
  }

  return _validFrame;
}

// Parsing NMEA
bool RTK_Common::parseNMEA(const uint8_t *buffer, size_t length) {
  String nmea = String(buffer, length);
  String message = "";
  
  if( checkNMEACRC( nmea ) )
  {
    // Trame NMEA valide    
    if( nmea.startsWith("$GNGGA") ) {
      message = "Fix (GGA): ";
      Serial.print( message );
      Serial.println( nmea );
      extractNMEACoordinate( nmea );
    } else if( nmea.startsWith("$GNRMC") ) {
      message = "Recommended Minimum (RMC): ";
    } else if( nmea.startsWith("$PQTMSVINSTATUS") ) {
      message = "PQTM > ";
      Serial.print( message );
      Serial.println( nmea );

      // Monitore le Survey-in process to complete (=> change to FixBAse Mode if success)
      surveyIn_ECEFCoordinate( nmea );
       // Inform Rover for Survey complete [optional]     
      if( _RTKmode == RTK_MODE::MODE_FIXBASE && _sendUDP ) {
          _sendUDP(buffer, length);  // délégué à l’extérieur             
      }
    } else if( nmea.startsWith("$PQTM") ) {
      message = "PQTM > ";
      Serial.print( message );
      Serial.println( nmea );
    } else if( nmea.startsWith("$PAIR") ) {
      message = "PAIR > ";
      Serial.print( message );
      Serial.println( nmea );
    } else {
      message = "Other NMEA: ";
    }
/*
    if( DEBUG_GPS_INFO )
    { // Debug Print NMEA
      Serial.print( message );
      Serial.println( nmea );
    }
*/
    // Autres types ici
    return true;
  }
  else
  { // Trame NMEA invalide
    invalidFrame( "Wrong CRC - NMEA > ", nmea.c_str() );
  }

  return false;
}

void RTK_Common::extractNMEACoordinate( const String nmea ) {
  if( nmea.length() < 6 ) return;

  // === Identifier le type de trame (GGA, RMC...) sans le préfixe GNSS ===
  String nType = nmea.substring(3, 6);

  // ==== EXTRACT LAT/LON ====
  if( (nmea.startsWith("$GNGGA")) || (nmea.startsWith("$GNRMC")) ) {
    // On découpe la trame en morceaux
    int fieldIndex = 0;
    String fields[20];  // max 20 champs
    int start = 0;

    for (int i = 0; i < nmea.length(); i++) {
      if( nmea.charAt(i) == ',' || nmea.charAt(i) == '*' ) {
        fields[fieldIndex++] = nmea.substring(start, i);
        start = i + 1;
      }
    }

    // GGA: $GNGGA,time,lat,N/S,lon,E/W,...
    // RMC: $GNRMC,time,status,lat,N/S,lon,E/W,...
    int fixIndex = 6;
    int latIndex = nType == "GGA"? 2 : 3;
    int lonIndex = latIndex + 2;

    String mesg = "$GPS,";
    if( _RTKmode == RTK_MODE::MODE_ROVER )
        mesg += "Rover,";
    else
        mesg += "_Base,";

    // === TIME EXTRACTION ===
    if( fields[1].length() >= 6 ) {
      String timeRaw = fields[1]; // e.g. "123519.00"
      String hour = timeRaw.substring(0, 2);
      String minute = timeRaw.substring(2, 4);
      String second = timeRaw.substring(4, 6);
      mesg += hour + ":" + minute + ":" + second + ",";
    }

    // Insert GPS Fix value
    if( nmea.startsWith("$GNGGA") && fields[fixIndex].length() > 0 ) {
      // Convertir en entier pour switch
      int code = fields[fixIndex].toInt();
      switch (code) {
        case 0: mesg += "0,NO Fix,"; break;
        case 1: mesg += "1,Fix GPS,"; break;
        case 2: mesg += "2,Fix DGPS,"; break;
        case 3: mesg += "3,Fix PPP,"; break;
        case 4: mesg += "4,Fix RTK,"; break;
        case 5: mesg += "5,Fix Float RTK,"; break;
        case 6: mesg += "6,Mode DR,"; break;
        case 7: mesg += "7,Manual Fix,"; break;
        case 8: mesg += "8,Simulator,"; break;
        case 9: mesg += "9,Fix WAAS,"; break;    // SBAS
        default: mesg += String(code) + ",Unknown,"; break;
      }
    }

    // Insert GPS latidue et longitude
    if( fields[latIndex].length() > 0 && fields[lonIndex].length() > 0 ) {
      _latitude  = convertNMEACoordinate(fields[latIndex], fields[latIndex + 1]);
      _longitude = convertNMEACoordinate(fields[lonIndex], fields[lonIndex + 1]);

      // Insert Lat,Lon:
      mesg += String(_latitude, 10) + ","  + String(_longitude, 10);
    }
    
    // Output GPS position data
    Serial.println( mesg );

    // Inform Rover of new GPS coordinate [optional]
    if( _RTKmode == RTK_MODE::MODE_FIXBASE && _sendUDP ) {
        _sendUDP( (const unsigned char*)mesg.c_str(), mesg.length() );
    }
  }
}

double RTK_Common::convertNMEACoordinate(String nmeaCoord, String direction) {
  if( nmeaCoord.length() < 4 ) return 0.0;

  // Exemple : 4807.038,N → 48°07.038'
  double raw = nmeaCoord.toDouble();
  int deg = int(raw / 100);
  double min = raw - (deg * 100.0);
  double dec = deg + (min / 60.0);

  if( direction == "S" || direction == "W" )
    dec = -dec;

  return dec;
}

bool RTK_Common::parseRTCM3(const uint8_t *buffer, size_t length) {
  if( length < 6 ) return false;  // trop court

  // Reprend la longueur depuis l'entête
  uint16_t rtcmLength = ((buffer[1] & 0x03) << 8) | buffer[2];
  
  // Calcul le CRC de la trame => L = 3 header + N + 3 CRC
  //Serial.println("LEN"); Serial.println(length); Serial.println(rtcmLength + 6);
  if( length == rtcmLength + 6 )
  { 
    uint32_t crcRx = (buffer[length - 3] << 16) |
                     (buffer[length - 2] << 8 ) |
                      buffer[length - 1];
    uint32_t crcCalc = computeCRC24Q( buffer, rtcmLength + 3 );
    //Serial.println("CRC"); Serial.println(crcRx); Serial.println(crcCalc);

    // Si la trame RTCM est valide
    if( crcRx == crcCalc )
    {
      if( DEBUG_GPS_INFO )
      {
        // Décalage pour le début des données après 3 octets d'en-tête
        const uint8_t *payload = &buffer[3];

        // Message type = bits 0 à 11
        uint16_t msgType = getBits(payload, 0, 12);
        //Serial.print("RTCM MsgType: ");
        //Serial.println(msgType);

        switch (msgType) {
          case 1005:
            //decodeRTCM1005(payload, length - 6);
            break;
          case 1077:
            //Serial.println("RTCM GNSS (1077) - not decoded here");
            break;
          default:
            //Serial.print("Msg RTCM (");
            //Serial.print(msgType);
            //Serial.print("), taille=");
            //Serial.println(length);
            break;
        }
      }

      Serial.print('.');
      return true;
    }
    else {
      invalidFrame( "RTCM - CRC FAIL - ", (const char*)buffer );
      //Serial.print(length);
      //Serial.print(' ');
      //Serial.print(crcRx);
      //Serial.print(' ');
      //Serial.println(crcCalc);
    }
  } else {
      invalidFrame( "RTCM - Invalid Length - ", (const char*)buffer );
      //Serial.println(length);
    }

  return false;
}

// Lecture bits utils RTCM3
uint32_t RTK_Common::getBits(const uint8_t *data, int startBit, int bitLen) {
  uint32_t result = 0;
  for (int i = 0; i < bitLen; i++) {
    int bit = startBit + i;
    int byteIdx = bit / 8;
    int bitIdx = 7 - (bit % 8);
    result <<= 1;
    result |= (data[byteIdx] >> bitIdx) & 0x01;
  }
  return result;
}

// ========== Décodage message RTCM 1005 (Station de base) ==========
void RTK_Common::decodeRTCM1005(const uint8_t *payload, size_t len) {
  uint16_t stationID = getBits(payload, 12, 12);
  uint8_t  system = getBits(payload, 24, 6);
  uint8_t  refStation = getBits(payload, 30, 1);
  int32_t  x = getBits(payload, 34, 38);
  int32_t  y = getBits(payload, 74, 38);
  int32_t  z = getBits(payload, 114, 38);

  Serial.println("RTCM 1005 - Station Reference");
  Serial.print(" - Station ID: "); Serial.println(stationID);
  Serial.print(" - ITRF System: "); Serial.println(system);
  Serial.print(" - Reference Station: "); Serial.println(refStation);
  Serial.print(" - ECEF X: "); Serial.println((double)x * 0.0001, 7);
  Serial.print(" - ECEF Y: "); Serial.println((double)y * 0.0001, 7);
  Serial.print(" - ECEF Z: "); Serial.println((double)z * 0.0001, 7);
}

