#include "RC_acquisition.h"
#include "pin_definition.h"

const int THRESHOLD = 100;              // Marge de seuil pour les positions de la télécommande (+ la marge augment + la valeur est forte)

unsigned long RC_value_min = 0;         // Valeur min de la plage de valeur télécommande
unsigned long RC_value_max = 1000;      // Valeur max de la plage de valeur télécommande

unsigned long RC_start_time = 1000;     // Temps à partir duquel on commence la lecture du temps d'impulsion télécommande
unsigned long RC_max_end_time = 1980;   // Temps max d'une impulsion télécommande lorsque levier à fond

unsigned long RC_value[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};      // Valeurs Etats des 8 channels de la RC
unsigned long RC_ref_timing[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Sauvegarde du timimg précédent (utilisé pour le clcul de la longueur d'impulsion RC)


void init_RC() {
  slog( 1, "_RC_", "init RC....................", false );
  pinMode(RC1_pin, INPUT);                               // Throttle
  attachInterrupt(RC1_pin, ISR_RC1_reading, CHANGE);
  pinMode(RC2_pin, INPUT);                               // ??
  attachInterrupt(RC2_pin, ISR_RC2_reading, CHANGE);
  pinMode(RC3_pin, INPUT);                               // ..
  attachInterrupt(RC3_pin, ISR_RC3_reading, CHANGE);
  pinMode(RC4_pin, INPUT);                               // Yaw control
  attachInterrupt(RC4_pin, ISR_RC4_reading, CHANGE);
  pinMode(RC5_pin, INPUT);
  attachInterrupt(RC5_pin, ISR_RC5_reading, CHANGE);
  pinMode(RC6_pin, INPUT);
  attachInterrupt(RC6_pin, ISR_RC6_reading, CHANGE);
  pinMode(RC7_pin, INPUT);
  attachInterrupt(RC7_pin, ISR_RC7_reading, CHANGE);
  pinMode(RC8_pin, INPUT);
  attachInterrupt(RC8_pin, ISR_RC8_reading, CHANGE);
  clog( "Done." );
  elog();
}

void read_RC()
{
  // Dummy function as all RC positions are read by Interrupts
}

void wait_RCMode( int mode )
{
  //--- Attendre tend que l'on ne passe pas en Mode 3 = Mode Manuel
  slog( 1, "_RC_", "Waiting for entering Mode ", false ); clog(mode); elog();
  while( mode != RC_value[7] ) //   + mode Wifi   && mode != Wifi_value[7] )
  { delay(100); }
  slog( 1, "_RC_", "Waiting Unlocked", true );
}

void display_RC()
{ /* Affichage des valeurs sur le terminal série pour verrifier les valeur du switch */
  slog( 1, "_RC_", "Telecommande :  ", false );
  for (int i = 1; i <= 8; i++)
  {   clog("RC"); clog( i ); clog(":"); clog( RC_value[i] ); clog(" | "); }
  elog();
}

void IRAM_ATTR ISR_RC1_reading() {  // Gauche vertical : CH3 reciever
  int channel = 1;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = constrained_map(RC_duration, RC_start_time, RC_max_end_time, RC_value_min, RC_value_max);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC2_reading() {  // Gauche horizontal : CH4 reciever
  int channel = 2;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = constrained_map(RC_duration, RC_start_time, RC_max_end_time, RC_value_min, RC_value_max);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC3_reading() {  // Droite vertical : CH2 reciever
  int channel = 3;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = constrained_map(RC_duration, RC_start_time, RC_max_end_time, RC_value_min, RC_value_max);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC4_reading() {  // Droite horizontal : CH1 reciever
  int channel = 4;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = constrained_map(RC_duration, RC_start_time, RC_max_end_time, RC_value_min, RC_value_max);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC5_reading() {  // Switch A : CH5 reciever
  int channel = 5;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = RC_Switch(RC_duration);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC6_reading() {  // Switch B : CH6 reciever
  int channel = 6;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = RC_Switch(RC_duration);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC7_reading() {  // Switch C : CH7 reciever
  int channel = 7;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = RC_Switch(RC_duration);
  }
  RC_ref_timing[channel] = RC_timing;  
}

void IRAM_ATTR ISR_RC8_reading() {  // Switch D : CH8 reciever
  int channel = 8;
  unsigned long RC_duration = 0;
  unsigned long RC_timing = 0;
  RC_timing = micros();
  if (RC_timing > RC_ref_timing[channel]) {
    RC_duration = RC_timing - RC_ref_timing[channel];
    if (RC_duration < 2500)
      RC_value[channel] = RC_Switch(RC_duration);
  }
  RC_ref_timing[channel] = RC_timing;  
}

unsigned long IRAM_ATTR RC_Switch(long RC_duration) {
  // Vers le bas
  if( RC_duration <= (1500 - THRESHOLD) )
    return 1;
  // Milieu
  else if( RC_duration >= (1500 + THRESHOLD) )
    return 3;
  // Vers le haut
  else
    return 2;

  return 0;
}

long IRAM_ATTR constrained_map(long x, long in_min, long in_max, long out_min, long out_max) {      // Même fonctionnement que map() mais garanti que le resultat est dans la plage de sortie indiquée
  long mapped_value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (mapped_value < out_min)
    return out_min;
  else if (out_max < mapped_value)
    return out_max;
  else
    return mapped_value;
}