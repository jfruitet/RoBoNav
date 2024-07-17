//*****************************************************************************************//
// RoboNav - Buyobot ARBL Flight/Swim Controller
// Copyright ICAM/ARBL 2022-2024
//*****************************************************************************************//
// Created, Maintener : Nicolas FERRY
// Autors ICAM 2023 : Marie Louvet, Enora Fremy
// Autors ICAM 2024 : Pierre-Louis Burger, Gauthier Ailleret
//*****************************************************************************************//
// Created:  NF, EF-ML
// Modified: RC_acquisition, initial GPS, Wifi_Acquisition - NF
// Modified: ESC_control - PL
// Modified: GPS, State-Machine - GA
// Modified: Refactoring - NF
//*****************************************************************************************//
#define ROBONAV_VERSION "1.2.3"

#include "pin_definition.h"
#include "utils.h"
#include "IO_control.h"
#include "ESC_control.h"
#include "RC_acquisition.h"
#include "GPS_acquisition.h"
#include "Wifi_acquisition.h"


//--- Variables de Control Debug du Système ------------------------------------------//
bool activerSerial = true;       
bool activerWifi = true;
uint32_t LOOP_REFRESH_RATE = 100;

//--- Constantes de Control NAV et Stabilisation  ------------------------------------//
const long  MAX_VELOCITY = 1800;      // Valeur PWM de vitesse Maximale de l'ESC entre 1500-2000
const long  MIN_VELOCITY = 1500;      // Valeur PWM de vitesse Minimale de l'ESC entre 1500-2000
const float RANGE_STOPMOVE = 1.0;     // Distance jusqu'à laquelle on laisse dériver
const float RANGE_RAMPUP   = 4.0;     // Distance en mètre pour atteindre la vitesse de croisière
const float RANGE_RAMPDOWN = 10.0;    // Distance en mètres avant décéleration (Regulation droite face au vent)
const float UPSIDE_ANGLE   = 10.0;    // Cône d'ouverture de l'angle de visé. Il faut compter 2x l'angle (-A et +A) par rapport au 0°


//--- Variables d'état globale -------------------------------------------------------//
bool setupCompleted = false;
bool newGPSFix = false;
int  SC_Mode = 1;                          // Valeur du Mode Switch actuel (Pilote la machine à état)
int  Prec_SC_Mode = 1;                     // Valeur précédente du mode pour détection des changements d'état
int  RC_SC_Mode = 1;                       // Valeur du switch point de telecommande RC
int  WiFi_SC_Mode = 1;                     // Valeur du switch point de vue WiFi

//--- Navigation Systeme -------------------------------------------------------------//
double lat_position_RTH = 0.0;
double lng_position_RTH = 0.0;
double lat_position_bouee = 0.0;
double lng_position_bouee = 0.0;
double lat_position_start = 0.0;
double lng_position_start = 0.0;
double lat_position_dest = 0.0;
double lng_position_dest = 0.0;
double lat_position_dest_initiale = 0.0;
double lng_position_dest_initiale = 0.0;
double lat_position_dest_test = 47.244061;
double lng_position_dest_test = -1.474036;

float angle_bouee;                          // Angle Compas => Plus précis que l'angle GPS pour l'angle bouée réel
float nav_angle;                            // Angle entre les points GPS (bouéee, cible)
float target_dist;                          // Distance juqu'à la cible (=nav_dist mais c'est pour la lisibilité)
float target_angle;                         // Angle à réaliser pour attenidre la cible (angle_GPS - Angle compas)
float total_dist;                           // Distance Absolu (à la création du chemin) n'évolue pas au cours du temps
float total_angle;                          // Angle Absolu (à la création du chemin) n'évolu pas au cours du temps

//--- Dynamic Synchro Loop -------------------------------------------------------------//
uint32_t start_refTime = 0;                 // used to calculate integration interval
uint32_t ended_refTime = 0;                 // used to calculate integration interval
uint32_t wait_refTime = 0;                 // used to calculate integration interval


//=============================================================================================================================================
//                    Power on - Setup
//=============================================================================================================================================
void setup()
{
   //--- Init Arduino Debug UART0 OR WiFi ---//
   init_Log( true, activerSerial, false ); // Attention le WiFi doit être initialisé AVANT de pouvoir utiliser le Log WiFi
   if( activerWifi )        
      init_WiFi();

   init_Log( true, activerSerial, activerWifi );
   slog( 0, "RoboNav", "Starting RoboNav Controller v", false ); clog( ROBONAV_VERSION ); elog();
   slog( 0, "RoboNav", "Copyright ICAM/ARBL 2022-2024", true );
   
   //--- Init Equipements ---------------------//
   init_IO();
   init_RC();
   init_ESC();
   init_GPS();
   init_Compass();

   //--- Calibrate Equipements ----------------//
   wait_RCMode(1);              // Wait dor RC Command in Mode 1 = Return To Home (before starting calibrate and GPS RTH Fix)
   calibrate_ESC();
   calibrate_Compass();
   wait_GPSFix( true );         // Wait for GPS Fix and Store it at Return to Home Position

   //--- Setup Completed ----------------------//
   wait_RCMode(3);              // Wait dor RC Command in Mode 3 = Mode Manuel (before starting)
   slog( 0, "INIT", "Setup Completed !", true );
   setupCompleted = true;
}



//=============================================================================================================================================
//                                                 MAIN LOOP
//=============================================================================================================================================
void loop()
{
  start_refTime = millis();
  
  //--- (1) Read RC & WiFi commands ----------------------------------------------------------//
  //slog( 1, "MAIN", "1- Read RC & WiFi", true );
  read_RC();
  display_RC();
  if( activerWifi )
  {
    radioDecode_WiFi();
    display_WiFi();
    // Check for WiFi_SC_Mode changed ?
    if( WiFi_SC_Mode != Wifi_value[7] )
      SC_Mode = Wifi_value[7];
  }
  //--- RC Switch Priority for Switch Mode Changing ----//
  if( RC_SC_Mode != RC_value[7] )
      SC_Mode = RC_value[7];

  //--- Update RC and WiFi Switch Value for next update ---//
  RC_SC_Mode = RC_value[7];
  WiFi_SC_Mode = Wifi_value[7];
  
  //--- Debug SC Mode Switch Value ---//
  slog( 2, "MAIN", "SC_Mode=", false ); clog( SC_Mode ); clog(" RC_Mode="); clog( RC_SC_Mode ); clog(" WiFi_Mode="); clog( WiFi_SC_Mode ); elog();

  //--- (2) Read Devices & Sensors -------------------------------------------------------------//
  //slog( 1, "MAIN", "2- Read Devices & Sensors (Compass, GPS...)", true );
  angle_bouee = read_Compass();
  newGPSFix = update_GPS();
  // Read others sensors here

  //--- (2b) Update Controls -------------------------------------------------------------------//
  update_LED();
  update_BUZZER();

  //--- (3) NAV Controller ---------------------------------------------------------------------//
  //slog( 1, "MAIN", "3- NAV Controller", true );
  if( newGPSFix )
      slog( 2, "NAV", "New GPS Position Updated", true ); 
      
  target_dist  = getGPSDistance( lat_position_bouee, lng_position_bouee, lat_position_dest, lng_position_dest );
  nav_angle    = getGPSAngle( lat_position_bouee, lng_position_bouee, lat_position_dest, lng_position_dest );
  target_angle = deltaAngle( angle_bouee, nav_angle );
  
  slog( 0, "NAV", "CAP=", false ); clog( angle_bouee ); clog("°");
    clog(" GPS="); clog( gps.location.lat() ); clog(","); clog( gps.location.lng() ); clog("  ");
    clog( "POS="); clog( lat_position_bouee ); clog(","); clog( lng_position_bouee ); clog("  ");
    clog( "NAV="); clog( lat_position_dest  ); clog(","); clog( lng_position_dest  ); //elog();
  //slog( 0, "NAV", "Target GPS-D=", false  ); 
  clog(" Target GPS-D="); clog( target_dist ); clog("m  GPS-A="); clog( nav_angle ); clog("°  A="); clog( target_angle ); clog("°"); elog();

  //--- State Machine Control Program --------------------------------------------------------//
  //slog( 1, "MAIN", "State Machine Control Program", true );
  if( RC_value[5] == 3 ) {
      stop_controlMotors();  
      slog( 0, "MAIN", "EMERGENCY STOP !", true );
  }
  else
  switch( SC_Mode )
  {
    //=========================================================================================================================================
    //                           ALLER EN POSITION
    //=========================================================================================================================================
    case 1: // Goto NAVpoint - Return to Home - Aller/Retour a une position donnée  (position SC haut)
    {
      slog( 1, "STATE", "case 1 - RETURN To HOME", true );

      //--- Is Entering Return To Home Mode ?? ---//
      if( Prec_SC_Mode != SC_Mode )
      { // Definition donnée de localisation
         slog( 0, "RTH", "TARGET Fixée à la position GPS RTH initiale.", true );
         lat_position_start = lat_position_bouee;
         lng_position_start = lng_position_bouee;
         lat_position_dest  = lat_position_RTH;
         lng_position_dest  = lng_position_RTH;
         lat_position_dest_initiale = lat_position_dest;
         lng_position_dest_initiale = lng_position_dest;
         total_dist  = getGPSDistance(lat_position_start, lng_position_start, lat_position_dest, lng_position_dest);
         total_angle = getGPSAngle(lat_position_start, lng_position_start, lat_position_dest, lng_position_dest);
         break;
      }
      
      //--- Deplacement BOUEE - Aller / Rester a une position donnée
      if( target_dist > (total_dist + 200.0) )
      {   // Erreur
          stop_controlMotors();
          slog( 0, "RTH", "Home Position too far far away - Error GPS.........X", true );
      }
      else
      if( abs(total_dist - target_dist) < RANGE_RAMPUP )
      {  //--- Phase démarrage ------------------//
         // orientation vers la destination
         if( target_angle > UPSIDE_ANGLE || target_angle < -UPSIDE_ANGLE )
            turn_controlMotors( target_angle );
         else
            // Si on est dans l'axe - on démarre droit
            rampUp_controlMotors( abs(total_dist - target_dist) );
      }
      else  // Distance_Parcourue >= RANGE_RAMPUP
      { //--- Phase Cruise navigation ---//  
        if( target_dist > RANGE_RAMPDOWN )
          setConsigne_controlMotors( target_angle, target_dist );
        //--- Phase Ralentissement ----//
        else if( target_dist > RANGE_STOPMOVE )
          rampDown_controlMotors( target_dist );
        else //--- Phase Arret - target_dist <= RANGE_STOPMOVE ---//
          stop_controlMotors();
      }

      break;
    }
   
    //=========================================================================================================================================
    //                       RESTER EN POSITION
    //=========================================================================================================================================
    case 2: // GOTO Position & Stabilize - Rester en position  (position SC neutre)
    {
      slog( 1, "STATE", "case 2 - STABILIZE", true );
      
      //--- Is Entering Return To Home Mode ?? ---//
      if( Prec_SC_Mode != SC_Mode )
      { // Definition donnée de localisation
         slog( 0, "STABILIZE", "TARGET Fixée à la position de Fixation ACTUELLE - LAT=", false ); clog(lat_position_bouee); clog(" LNG="); clog( lng_position_bouee ); elog();
         lat_position_start = lat_position_bouee;
         lng_position_start = lng_position_bouee;
         lat_position_dest  = lat_position_bouee;
         lng_position_dest  = lng_position_bouee;
         lat_position_dest_initiale = lat_position_dest;
         lng_position_dest_initiale = lng_position_dest;
         break;
      }
      
      //--- Deplacement BOUEE - Aller / Rester a une position donnée
      if( target_dist <= RANGE_STOPMOVE )           //--- Distance de Déclenchement ---//
          stop_controlMotors();
      else if( target_dist <= RANGE_RAMPDOWN )      //--- Distance Ajustement fine face au vent ---//
          rampDown_controlMotors( target_dist );
      else if( target_dist <= 2000.0 )              //--- Régulation en orientation et vitesse de la bouée ---//
         // orientation vers la destination
         if( target_angle > UPSIDE_ANGLE || target_angle < -UPSIDE_ANGLE )
            turn_controlMotors( target_angle );
         else
            // Si on est dans l'axe - on régule jusqu'à la cible
            setConsigne_controlMotors( target_angle, target_dist );
      else
      {   // Erreur
          stop_controlMotors();
          slog( 0, "STABILIZE", "TARGET Position too far far away - Error GPS.........X", true );
      }

      break;
    }
   
    //=========================================================================================================================================
    //                            MANUEL
    //=========================================================================================================================================
    case 3: // MANual Mode - Contrôle Manuel  (position SC bas)
    {
      slog( 1, "STATE", "case 3 - MANUEL MODE", true );

      if( RC_value[8] == 3 )  // RC buoton bas droit vers le Haut = WiFI Command, vers le BAS = RC commande
      {
         throttle = map(Wifi_value[1], RC_value_min, RC_value_max, ESC_NEUTRAL, ESC_MAX);
         rotation = map(Wifi_value[4], RC_value_min, RC_value_max, -1 * rotation_sensitivity, rotation_sensitivity); // Négatif full gauche et positif full droite
      }
      else
      {
         throttle = map(RC_value[1], RC_value_min, RC_value_max, ESC_NEUTRAL, ESC_MAX);
         rotation = map(RC_value[4], RC_value_min, RC_value_max, -1 * rotation_sensitivity, rotation_sensitivity);
      }
      slog( 3, "MANU", "Throttle = ", false ); clog( throttle ); clog( " , Direction = " ); clog( rotation ); elog();

      //--- Control des Moteurs M1 & M2 ---//
      consigne_m1 = throttle - rotation - correc_rotation;
      consigne_m2 = throttle + rotation + correc_rotation;
    
      post_consigne_m1 = m1_control(consigne_m1);
      post_consigne_m2 = m2_control(consigne_m2);
      break;
    }
  } 
  // end of switch

  //--- Memorize actual SC_Mode => as now the Prec_SC_Mode (Detection des changement d'états)
  Prec_SC_Mode = SC_Mode;

  // Calibrage de la fréquence de la boucle principale
  ended_refTime = millis();
  if( (ended_refTime - start_refTime) < LOOP_REFRESH_RATE )
      wait_refTime = LOOP_REFRESH_RATE - (ended_refTime - start_refTime);
  else
      wait_refTime = 0;
  cpu_waitTime( wait_refTime );

  slog( 3, "MAIN", "------------------------------------------------------------------------------------", true);
  slog( 3, "MAIN", "--- REFRESH RATE : ", false); clog(LOOP_REFRESH_RATE); clog(" SYNC WAIT TIME:"); clog(wait_refTime); clog(" ----------------------------------------"); elog();
  slog( 3, "MAIN", "------------------------------------------------------------------------------------", true);
}