#include "GPS_acquisition.h"

#define NB_GPS_MOY  64

TinyGPSPlus gps;
HMC5883L compass;

double posLat[NB_GPS_MOY];
double posLng[NB_GPS_MOY];
double filterCoef_GPS[5] = { 0.6, 0.15, 0.1, 0.1, 0.05 };

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
    slog( 1, "GPS", "init GPS...................", false );
    //--- initialisation UART2 - GPS UART -----------------------//
    Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RxPin, GPS_TxPin);

    clog("Done.");
    elog();
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
/*  char c = 0xFF;
  while( Serial2.available() > 0 )
  { c = Serial2.read();
    if( c == '$' ) {
        Serial.print(c);
        gps.encode( c );
        break;
    }
  }*/

  // This loop read a line and does not block by returning to the main loop
  // if not gps command has not been issued yet
  char c = 0xFF;
  while( Serial2.available() > 0 )
  {
    c = Serial2.read();
    //Serial.print(c);
    res = gps.encode( c );
    if( res ) {
        //Serial.println();
        break;
    }
  }
  
  //--- Do if New GPS valid position updated ------------------------------------------------------//
  res = gps.location.isValid() && gps.location.isUpdated();
  if( res )
  {    
    float deltaGPS = getGPSDistance( lat_position_moy, lng_position_moy, gps.location.lat(), gps.location.lng() );
    Serial.print( deltaGPS ); Serial.print(" "); Serial.print( lat_position_moy, 6 ); Serial.print(" "); Serial.println(lng_position_moy, 6 );

    // Si c'est le 1er update => on valide la Lat et Lng PREC
    if( deltaGPS > 1.5 )
    {
        // Fixe comme étant la nouvelle position bouée
        lat_position_bouee = gps.location.lat();
        lng_position_bouee = gps.location.lng();
        
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
        posLat[0] = gps.location.lat();
        posLng[0] = gps.location.lng();

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
        lat_position_bouee = gps.location.lat();
        lng_position_bouee = gps.location.lng();

        double delta_Lat = gps.location.lat() - lat_position_prev;
        double delta_Lng = gps.location.lng() - lng_position_prev;
        
        // Decale la Target de la même erreur //
        lat_position_dest += delta_Lat;
        lng_position_dest += delta_Lng;

        //--- Store la nouvelle position comme la nouvelle (previous au tour d'après) ---//
        lat_position_prev = gps.location.lat();
        lng_position_prev = gps.location.lng();
    }
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
    lat_position_RTH = gps.location.lat();
    lng_position_RTH = gps.location.lng();
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
