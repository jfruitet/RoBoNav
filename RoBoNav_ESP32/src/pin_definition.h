/**************************************
RoBoNav - 2023 - 2024 
pin_definition.h
***************************************/

#ifndef pin_definition_h 
#define pin_definition_h

//--- RC Telecommande Pins Reading -------------------------------------------//
#define RC1_pin  26   // Gauche vertical : CH3 reciever
#define RC2_pin  25   // Gauche horizontal : CH4 reciever
#define RC3_pin  27   // Droite vertical : CH2 reciever
#define RC4_pin  14   // Droite horizontal : CH1 reciever
#define RC5_pin  33   // Switch A : CH5 reciever
#define RC6_pin  32   // Switch B : CH6 reciever
#define RC7_pin  35   // Switch C : CH7 reciever
#define RC8_pin  34   // Switch D : CH8 reciever

//--- RELAY Control / ESC Control ON/OFF and Config Pins ---------------------//
#define ESC12_ON  17  // ESC 1&2 - Activation ON/OFF control
#define ESC12_SET 19  // ESC 1&2 - Set Pulse to change configuration
#define ESC34_ON  16  // ESC 3&4 - (No Used for RoboNav)
#define ESC34_SET 18  // ESC 3&4 - (No Used for RoboNav)

//--- MOTORS PWM Output control Pins -----------------------------------------//
#define PWM_M1 12     // M1 - Moteur gauche
#define PWM_M2 13     // M2 - Moteur droit
#define PWM_M3 2      // M3 - (No Used for RoboNav)
#define PWM_M4 4      // M4 - (No Used for RoboNav)

//--- STATE and Indicator control Pins ---------------------------------------//
#define LED    15     // LED indicator ouput
#define BUZZER 5      // BUZZER Sound output

//--- DEVICES I2C & UART(GPS) control Pins -----------------------------------//
#define I2C_SCL 22    // I2C Clock
#define I2C_SDA 21    // I2C Data
#define GPS_Tx  16    // Serial UART2 (Attention peut être inversion)
#define GPS_Rx  17    // Serial UART2

//--- BAT MONITORING UBat & IBat state reading Pins -------------------------//
#define UBat 36       // Voltage Monitoring (check your BEC module for support)
#define IBat 39       // Current Monitoring (check your BEC module for support)


#endif