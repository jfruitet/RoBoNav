/**************************************
RoBoNav - 2023 - 2024 
RC_acquisition.h
ATTENTION : Ce code suppose que la radiocommande et le récepteur puissent gérer au moins 8 canaux
Consulter le document RoBoNav - Mise en oeuvre.pdf du dossier ../../Documentation 
***************************************/

#ifndef RC_acquisition_H
#define RC_acquisition_H

#include <Arduino.h>
#include "utils.h"

extern unsigned long RC_value_min;         // Valeur min de la plage de valeur télécommande
extern unsigned long RC_value_max;         // Valeur max de la plage de valeur télécommande

extern unsigned long RC_value[9];         // Valeurs Etats des 8 channels de la RC
extern unsigned long RC_ref_timing[9];    // Sauvegarde du timimg précédent (utilisé pour le clcul de la longueur d'impulsion RC)


unsigned long IRAM_ATTR RC_Switch(long RC_duration);
long IRAM_ATTR constrained_map(long x, long in_min, long in_max, long out_min, long out_max);

void init_RC();
void read_RC();
void wait_RCMode( int mode );
void display_RC();

//--- Interrupts Handlers ---//
void IRAM_ATTR ISR_RC1_reading();   // Throttle
void IRAM_ATTR ISR_RC2_reading();
void IRAM_ATTR ISR_RC3_reading();   // Yaw ontrol
void IRAM_ATTR ISR_RC4_reading();
void IRAM_ATTR ISR_RC5_reading();
void IRAM_ATTR ISR_RC6_reading();
void IRAM_ATTR ISR_RC7_reading();
void IRAM_ATTR ISR_RC8_reading();

#endif
