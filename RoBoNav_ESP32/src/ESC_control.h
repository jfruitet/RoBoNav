/*******************************************
RoBoNav  2023 - 2024 - ESC_control.h 

********************************************/

#ifndef ESC_control_H
#define ESC_control_H

#include "pin_definition.h"
#include "ESP32Servo.h"
#include "RC_acquisition.h"
#include "utils.h"

#define ESC_MIN     1000              // Temps d'impulsion correspondant à full backward - reverse
#define ESC_NEUTRAL 1500              // Temps d'impulsion correspondant à neutral - neutre
#define ESC_MAX     2000              // Temps d'impulsion correspondant à full forward - avance

const int correc_rotation = 0;        // Mode Manuel recentrage - Négatif vers la gauche et positif vers la droite   (REGLAGE)
const int rotation_sensitivity = 200; // Mode Manuel sensibilité de la rotation (REGLAGE)

extern const long  MAX_VELOCITY;
extern const long  MIN_VELOCITY;
extern const float RANGE_STOPMOVE;     // Distance jusqu'à laquelle on laisse dériver
extern const float RANGE_RAMPUP;       // Distance en mètre pour atteindre la vitesse de croisière
extern const float RANGE_RAMPDOWN;     // Distance en mètres avant décéleration


extern int consigne_m1;      // Consigne 1 avant fonction commande
extern int consigne_m2;      // Consigne 2 avant fonction commande
extern int post_consigne_m1; // Consigne réelle ESC1 µs
extern int post_consigne_m2; // Consigne réelle ESC2 µs

extern int throttle;        // Gaz
extern int rotation;        // Rotation vers la gauche quand min et vers la droite quand max


void init_ESC();
void calibrate_ESC();

void stop_controlMotors();
void turn_controlMotors( float nav_angle );
void rampUp_controlMotors( float nav_dist );
void rampDown_controlMotors( float nav_dist );
void setConsigne_controlMotors( float angle, float distance );

int  m1_control(int consigne);
int  m2_control(int consigne);

#endif
