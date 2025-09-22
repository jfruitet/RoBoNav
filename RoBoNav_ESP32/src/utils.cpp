/**************************************
RoBoNav - 2023 - 2025 
utils.cpp
***************************************/

#include "utils.h"

// DELAI NON BLOQUANT
bool tempsDepassee = false;
uint32_t refTime = 0;

bool log_enable = true;              // false au démarrage => init_Log( true ) doit activer le LOG
int  log_priority = 5;                // Log priotity for next messages (initialisé par slog)
bool log_Serial_activated = false;    // false au démarrage
bool log_WiFi_activated = false;      // false au démarrage


void init_Log( bool enable, bool serialLog , bool wifiLog )
{
  if( !serialLog )
      Serial.end();

  if( serialLog && !log_Serial_activated )
      Serial.begin( LOG_BAUDRATE );

  //--- Update Log state ---//
  log_enable = enable;
  log_Serial_activated = serialLog;
  log_WiFi_activated = wifiLog;
}

void slog( int priority, const char* topic, const char* message, bool endLine )
{
    log_priority = priority;
    if( log_enable && log_priority <= LOG_MAX_LEVEL ) {
      if( log_Serial_activated ) {
          Serial.print( millis() ); Serial.print(" ["); Serial.print(topic); Serial.print("] "); Serial.print( message );
          if( endLine ) {
              Serial.println();
              Serial.flush();
          }
      }
      if( log_WiFi_activated ) {
          udp.beginPacket( gateway, UDP_PORT );
          udp.print( millis() ); udp.print(" ["); udp.print(topic); udp.print("] "); udp.print( message );
          if( endLine ) {
              udp.println();
              udp.endPacket();
          }
      }
    }
}

void clog( const char* message )
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL )
    {
      if( log_Serial_activated )
          Serial.print( message );
      if( log_WiFi_activated )
          udp.print( message );
    }
}

void clog( int value )
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL )
    {
      if( log_Serial_activated )
          Serial.print( value );
      if( log_WiFi_activated )
          udp.print( value );
    }
}

void clog( uint32_t value )
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL )
    {
      if( log_Serial_activated )
          Serial.print( value );
      if( log_WiFi_activated )
          udp.print( value );
    }
}

void clog( long value )
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL )
    {
      if( log_Serial_activated )
          Serial.print( value );
      if( log_WiFi_activated )
          udp.print( value );
    }
}

void clog( float value )
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL )
    {
      if( log_Serial_activated )
          Serial.print( value );
      if( log_WiFi_activated )
          udp.print( value );
    }
}

void clog( double value )
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL )
    {
      if( log_Serial_activated )
          Serial.print( value, 6 );
      if( log_WiFi_activated )
          udp.print( value, 6 );
    }
}

void elog()
{
    if( log_enable && log_priority <= LOG_MAX_LEVEL ) {
      if( log_Serial_activated ) {
          Serial.println();
          Serial.flush();
      }
      if( log_WiFi_activated ) {
          udp.println();
          udp.endPacket();
      }
    }
}


uint32_t start_RefTime()
{
    refTime = millis();
    return refTime;
}

bool is_elapsedTime( uint32_t delay )
{
    return tempsDepassee = ((millis() - refTime) >= delay);
}

void cpu_waitTime( uint32_t delay )
{
    refTime = millis();    
    while( tempsDepassee = ((millis() - refTime) < delay) )
    { yield(); }
}
