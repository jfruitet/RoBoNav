/**************************************
RoBoNav - 2025 - RTK.h
Développé par Agathe.D, intégré à RoBoNav par N. Ferry - ICAM Nantes
Module GPS RTK Quectel 
***************************************/


// ======================= RTK.h =======================
#ifndef RTK_MODULE_H
#define RTK_MODULE_H

#include <Arduino.h>

#define MAX_BUFFER 1024
#define DEBUG_GPS_INFO false

// --- GPS (HardwareSerial) ---
const byte rxPin = 16;  // ESP32-devKit = 16, ESP32-PoE-IND-ISO = 36
const byte txPin = 17;  // ESP32-devKit = 17, ESP32-PoE-IND-ISO = 4


enum class RTK_MODE : uint8_t {
    MODE_PT = 0,        // simple pass-through GNSS <-> host
    MODE_ROVER = 1,     // RTK Rover
    MODE_FIXBASE = 2,   // Station de base fixe ECEF
    MODE_SURVEYIN = 3   // Station de base "Survey-in"
};

//--- Procédure d'envoi des données par une fonction externe ---//
typedef void (*SendUDPCallback)(const uint8_t* data, size_t len);




// ======================= Classe commune =======================
class RTK_Common
{
  protected:
    HardwareSerial* _gps;     // Serial(0,1,2..) - HardwareSerial
    SendUDPCallback _sendUDP; // Procédure to forwardRTCM information (Base mainly)

    RTK_MODE _RTKmode = RTK_MODE::MODE_PT;
    uint8_t _modeFix = 0;
    uint8_t _nbSatellites = 0;
    bool    _receivingRTCM = false;
    bool    _receivingNMEA = false;
    bool    _validFrame = false;

    double  _latitude = 0.0;
    double  _longitude = 0.0;

  protected:
    uint16_t _bufferIndex = 0;
    uint8_t  _rawBuffer[MAX_BUFFER+1];
    uint32_t _reserved = 0;

  protected:
    virtual void initBase_ECEFCoord( double x, double y, double z ) {}
    virtual void surveyIn_ECEFCoordinate( String nmea ) {}

    // CRC NMEA
    String addNMEAchecksum(const String& message);
    bool checkNMEACRC(const String &nmea);

    // CRC RTCM
    uint32_t computeCRC24Q(const uint8_t *data, size_t length);

    // Parsing
    bool parseNMEA(const uint8_t *buffer, size_t length);
    bool parseRTCM3(const uint8_t *buffer, size_t length);
    bool invalidFrame( const char *err_msg, const char *buffer );
    void frameDebug_Display();
    
    // Extraction utils
    void extractNMEACoordinate(const String nmea);
    double convertNMEACoordinate(String nmeaCoord, String direction);
    uint32_t getBits(const uint8_t *data, int startBit, int bitLen);
    void decodeRTCM1005(const uint8_t *payload, size_t len);


  public:
    RTK_Common(HardwareSerial& gpsStream) : _gps(&gpsStream) { _sendUDP = nullptr; }
    void setSendUDPCallback(SendUDPCallback cb) { _sendUDP = cb; }  // Procedure d'affectation du callback SendUDP externe
    
    void transparentMode( bool mode );    // Place GNSS in transparent mode - Direct2Console GNSS link - WARNING Create an infinite loop for debug purpuse
    void resetGPS();                      // Reset du module (reboot duration fixed at 2s)
    bool update();                        // Routine de lecture des frames du modules GNSS (à appeler régulièrement)

    void getGPScoordinates( double &latitude, double &longitude );
    void inject_NMEA(const String& nmea); // Injection de commande NMEA externe (ex: commande utilisateur)
};


// ======================= Classe Base =======================
class RTK_Base : public RTK_Common
{
  public:
    RTK_Base(HardwareSerial& gps) : RTK_Common(gps) { _sendUDP = nullptr; }

    void begin( bool transparent );           // configure le module en mode Base RTK
    void configureGPS_BaseRTK( RTK_MODE mode, double x, double y, double z );

  protected:
    void initBase_ECEFCoord( double x, double y, double z ) override;
    void surveyIn_ECEFCoordinate( String nmea ) override;
};


// ======================= Classe Rover =======================
class RTK_Rover : public RTK_Common
{
  public:
    RTK_Rover(HardwareSerial& gps) : RTK_Common(gps) { _sendUDP = nullptr; }

    void begin( bool transparent ); // configure le module en mode Rover RTK

    // Injection du flux RTCM de correction de l'exterieur vers le Rover, provenance de la base (Radio UDP, centipede, etc.)
    void inject_RTCM(const uint8_t *data, size_t len);
};

#endif // RTK_MODULE_H
