/***************************************
RoBoNav  2023 - 2025 - ESC_control.cpp 
Contrôle  des moteurs
Version développée pour les ESC réversible HobbyWing QuicRun WP 10BL60 60 G2
A adapter aux ESC réversibles ZMR 40A qui n'ont pas exactement la même programmation.
Modifier éventuellement les fonctions 
    int m1_control( int consigne ); // Moteur gauche
    int m1_control( int consigne ); // Moteur droit

****************************************/


#include "ESC_control.h"

int consigne_m1 = 0;                  // Consigne 1 avant fonction commande
int consigne_m2 = 0;                  // Consigne 2 avant fonction commande
int post_consigne_m1 = 0;             // Consigne réelle ESC 1 (en µs)
int post_consigne_m2 = 0;             // Consigne réelle ESC 2 (en µs)

int throttle = 0;                     // Mode Manuel - Gaz
int rotation = 0;                     // Mode Manuel - Rotation (vers la gauche quand min et vers la droite quand max)

Servo ESC_MoteurG;                    // ESC Moteur Gauche - 1
Servo ESC_MoteurD;                    // ESC Moteur Droit - 2

unsigned long half_neutral_range = 30; //  Défini la plage de neutre de l'esc : 30µs de chaque côté de la valeur centrale

unsigned long refTime_reverse_m1 = 0;
unsigned long reverse_duration_m1 = 0;
bool time_stopped_m1 = true;

unsigned long refTime_reverse_m2 = 0;
unsigned long reverse_duration_m2 = 0;
bool time_stopped_m2 = true;


void init_ESC()
{
    slog( 1, "ESC", "init ESC...................", false );

    //--- Initialise ESC -------------------------------------//
    pinMode(PWM_M1, OUTPUT);
    ESC_MoteurG.attach(PWM_M1, ESC_MIN, ESC_MAX);
    ESC_MoteurG.writeMicroseconds(ESC_NEUTRAL);
    
    pinMode(PWM_M2, OUTPUT);
    ESC_MoteurD.attach(PWM_M2, ESC_MIN, ESC_MAX);
    ESC_MoteurD.writeMicroseconds(ESC_NEUTRAL);
/*
    pinMode(PWM_M3, OUTPUT);
    ESC_Moteur3.attach(PWM_M3, ESC_MIN, ESC_MAX);
    ESC_Moteur3.writeMicroseconds(ESC_NEUTRAL);

    pinMode(PWM_M4, OUTPUT);
    ESC_Moteur4.attach(PWM_M4, ESC_MIN, ESC_MAX);
    ESC_Moteur4.writeMicroseconds(ESC_NEUTRAL);
*/
  clog("Done.");
  elog();
}

void calibrate_ESC()
{
  //--- Wait for ESC calibration on RC -----------------------//
}

void stop_controlMotors()
{
    //--- Stop Motors ----//
    post_consigne_m1 = m1_control( ESC_NEUTRAL );
    post_consigne_m2 = m2_control( ESC_NEUTRAL );
}

void turn_controlMotors( float nav_angle )
{
    //--- Tourner la bouée ---//
    if( nav_angle > 0.0 )
    {
        post_consigne_m1 = m1_control( (MAX_VELOCITY - MIN_VELOCITY) / 8 + MIN_VELOCITY );
        post_consigne_m2 = m2_control( MIN_VELOCITY );
    }
    else
    {
        post_consigne_m1 = m1_control( MIN_VELOCITY );
        post_consigne_m2 = m2_control( (MAX_VELOCITY - MIN_VELOCITY) / 8 + MIN_VELOCITY );
    }
}

void rampUp_controlMotors( float nav_dist )
{
    //--- Faire une accelération progressive ---//
    consigne_m1 = map( nav_dist, 0.0, RANGE_RAMPUP, MIN_VELOCITY, MAX_VELOCITY );
    consigne_m2 = consigne_m1;
    post_consigne_m1 = m1_control( consigne_m1 );
    post_consigne_m2 = m2_control( consigne_m2 );
}

void rampDown_controlMotors( float nav_dist )
{
    //--- Faire une accelération progressive ---//
    consigne_m1 = map( nav_dist, RANGE_STOPMOVE, RANGE_RAMPDOWN, MIN_VELOCITY, MAX_VELOCITY ); // /2 pour laisser u peu de marge à l'inertie de retour
    consigne_m2 = consigne_m1;
    post_consigne_m1 = m1_control( consigne_m1 );
    post_consigne_m2 = m2_control( consigne_m2 );
}

void setConsigne_controlMotors( float angle, float distance )
{
    float velocity;

    if( distance > RANGE_RAMPDOWN )
        velocity = MAX_VELOCITY;
    else
        velocity = distance * MAX_VELOCITY / RANGE_RAMPDOWN;

    //--- Proportionality Map from Velocity and Angle parameters ---//
    //--- Assume M1 LeftMotor et M2 RightMotor ---//
    if( angle > 0.0 ) {
        consigne_m1 = map( angle, 0, +90, velocity, MIN_VELOCITY );
        consigne_m2 = map( angle, 0, +90, velocity, MAX_VELOCITY );
    }
    else {
        consigne_m1 = map( angle, 0, -90, velocity, MAX_VELOCITY );
        consigne_m2 = map( angle, 0, -90, velocity, MIN_VELOCITY );
    }

    //--- Send Modtors Control -----------------------------------//
    post_consigne_m1 = m1_control( consigne_m1 );
    post_consigne_m2 = m2_control( consigne_m2 );
}



int m1_control( int consigne )
{ // Envoi de la consigne au moteur 1 - param[1000;2000]
  slog( 3, "MOTORS", "M1 = ", false ); clog( consigne );
  consigne = constrain(consigne, ESC_MIN, ESC_MAX);
  if (consigne >= ESC_NEUTRAL + half_neutral_range)
  { // Consigne plage marche avant
    ESC_MoteurG.writeMicroseconds(consigne);
    time_stopped_m1 = true;
    return consigne;
  }
  else if (ESC_NEUTRAL - half_neutral_range < consigne && consigne < ESC_NEUTRAL + half_neutral_range)
  { // Consigne plage neutre
    ESC_MoteurG.writeMicroseconds(ESC_NEUTRAL);
    time_stopped_m1 = true;
    return ESC_NEUTRAL;
  }
  else if (consigne <= ESC_NEUTRAL - half_neutral_range)
  { // Consigne plage reverse
    if (time_stopped_m1)
    { // Premier passage reverse
      ESC_MoteurG.writeMicroseconds(ESC_NEUTRAL);
      refTime_reverse_m1 = millis();
      time_stopped_m1 = false;
      return ESC_NEUTRAL;
    }
    else
    { // Pas premier passage reverse
      reverse_duration_m1 = millis() - refTime_reverse_m1;
      if (reverse_duration_m1 > 200)
      { // Timing > 200 ms
        ESC_MoteurG.writeMicroseconds(consigne);
        return consigne;
      }
      else if (reverse_duration_m1 > 100)
      { // Timing > 100 ms
        ESC_MoteurG.writeMicroseconds(ESC_NEUTRAL);
        return ESC_NEUTRAL;
      }
      else if (reverse_duration_m1 <= 100)
      { // Timing <= 100 ms
        ESC_MoteurG.writeMicroseconds(ESC_MIN);
        return ESC_MIN;
      }
    }
  }
  return 0;
}

int m2_control( int consigne )
{ // Insure m2_control AFTER m1_control (for log)
  clog("  M2 = "); clog( consigne ) ;elog();
  
  // Envoi de la consigne au moteur 2 - param[1000;2000]
  consigne = constrain(consigne, ESC_MIN, ESC_MAX);
  if (consigne >= ESC_NEUTRAL + half_neutral_range)
  { // Consigne plage marche avant
    ESC_MoteurD.writeMicroseconds(consigne);
    time_stopped_m2 = true;
    return consigne;
  }
  else if (ESC_NEUTRAL - half_neutral_range < consigne && consigne < ESC_NEUTRAL + half_neutral_range)
  { // Consigne plage neutrej
    ESC_MoteurD.writeMicroseconds(ESC_NEUTRAL);
    time_stopped_m2 = true;
    return ESC_NEUTRAL;
  }
  else if (consigne <= ESC_NEUTRAL - half_neutral_range)
  { // Consigne plage reverse
    if (time_stopped_m2)
    { // Premier passage reverse
      ESC_MoteurD.writeMicroseconds(ESC_NEUTRAL);
      refTime_reverse_m2 = millis();
      time_stopped_m2 = false;
      return ESC_NEUTRAL;
    }
    else
    { // Pas premier passage reverse
      reverse_duration_m2 = millis() - refTime_reverse_m2;
      if (reverse_duration_m2 > 200)
      { // Timing > 200 ms
        ESC_MoteurD.writeMicroseconds(consigne);
        return consigne;
      }
      else if (reverse_duration_m2 > 100)
      { // Timing > 100 ms
        ESC_MoteurD.writeMicroseconds(ESC_NEUTRAL);
        return ESC_NEUTRAL;
      }
      else if (reverse_duration_m2 <= 100)
      { // Timing <= 100 ms
        ESC_MoteurD.writeMicroseconds(ESC_MIN);
        return ESC_MIN;
      }
    }
  }
  return 0;
}
