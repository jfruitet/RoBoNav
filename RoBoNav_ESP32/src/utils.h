/**************************************
RoBoNav - 2023 - 2025 
utils.h
***************************************/

#ifndef utils_h
#define utils_h

#define LOG_BAUDRATE 115200
#define LOG_MAX_LEVEL 9

#include <Arduino.h>
#include "Wifi_acquisition.h"

extern bool log_enable;
extern bool activerWifi;

void init_Log( bool enable, bool serialLog , bool wifiLog );
void slog( int priority, const char* topic, const char* message, bool endLine );
void clog( const char* message );
void clog( int value );
void clog( unsigned long value );
void clog( uint32_t value );
void clog( float value );
void clog( double value );
void elog();


uint32_t start_RefTime();
bool is_elapsedTime( uint32_t delay );
void cpu_waitTime( uint32_t delay );

#endif
