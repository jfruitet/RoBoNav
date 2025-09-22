/***************************************
RoBoNav  2023 - 2024 - IO_control.cpp
Initialise les sortie LED, Buzzer et ESC des moteurs
****************************************/


#include "IO_control.h"


uint32_t nbImpulse_LED;
uint32_t refLedDelay;
uint32_t refLedTime;

uint32_t nbImpulse_BUZZER;
uint32_t refBuzDelay;
uint32_t refBuzTime;


void init_IO()
{
    slog( 1, "_IO_", "init I/O...................", false );

    //--- Initialize IO controls : Led, Buzzer, 4x RElays
    pinMode( LED, OUTPUT );
    pinMode( BUZZER, OUTPUT );

    pinMode( ESC12_ON,  OUTPUT );
    pinMode( ESC12_SET, OUTPUT );
    pinMode( ESC34_ON,  OUTPUT );
    pinMode( ESC34_SET, OUTPUT );

    digitalWrite( LED, LOW );
    digitalWrite( BUZZER, LOW );

    digitalWrite( ESC12_ON,  LOW );
    digitalWrite( ESC12_SET, LOW );
    digitalWrite( ESC34_ON,  LOW );
    digitalWrite( ESC34_SET, LOW );

    clog("Done.");
    elog();
}

bool read_StateIO( int pin )
{
    return digitalRead( pin );
}

void write_StateIO( int pin, bool value )
{
    digitalWrite( pin, value?HIGH:LOW );
}


void bright_LED( uint32_t delay ) 
{
    write_StateIO( LED, HIGH );
    nbImpulse_LED = 1;
    refLedDelay = delay;
    refLedTime = millis();
}

void blink_LED( uint8_t nbTime, uint32_t delay )
{    
    if( nbTime <= 0 )
        return;

    write_StateIO( LED, HIGH );
    nbImpulse_LED = nbTime * 2;
    refLedDelay = delay / nbImpulse_LED;
    refLedTime = millis();
}

void update_LED()
{
    if( nbImpulse_LED > 0 )
    {
        if( (millis() - refLedTime) >= refLedDelay )
        {
            nbImpulse_LED--;
            if( nbImpulse_LED > 0 ) {
                write_StateIO( LED, read_StateIO( LED ) ^ 1 );    // Echange l'état de la LED
                refLedTime = millis();
            }
            else
                write_StateIO( LED, LOW );                        // Eteindre la LED
        }
    }
}

void tone_BUZZER( uint32_t delay )
{
    write_StateIO( BUZZER, HIGH );
    nbImpulse_BUZZER = 1;
    refBuzDelay = delay;
    refBuzTime = millis();
}


void ring_BUZZER( uint8_t nbTime, uint32_t delay )
{    
    if( nbTime <= 0 )
        return;

    write_StateIO( BUZZER, HIGH );
    nbImpulse_BUZZER = nbTime * 2;
    refBuzDelay = delay / nbImpulse_BUZZER;
    refBuzTime = millis();
}

void update_BUZZER()
{
    if( nbImpulse_BUZZER > 0 )
    {
        if( (millis() - refBuzTime) >= refBuzDelay )
        {
            nbImpulse_BUZZER--;
            if( nbImpulse_BUZZER > 0 ) {
                write_StateIO( BUZZER, read_StateIO( BUZZER ) ^ 1 );    // Echange l'état de la LED
                refBuzTime = millis();
            }
            else
                write_StateIO( BUZZER, LOW );                        // Eteindre la LED
        }
    }
}
