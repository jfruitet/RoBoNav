#ifndef IO_control_H
#define IO_control_H

#include "pin_definition.h"
#include "utils.h"


void init_IO();

bool read_StateIO( int pin );
void write_StateIO( int pin, bool value );

void bright_LED( uint32_t delay );
void blink_LED( uint8_t nbTime, uint32_t delay );
void update_LED();

void tone_BUZZER( uint32_t delay );
void ring_BUZZER( uint8_t nbTime, uint32_t delay );
void update_BUZZER();

#endif