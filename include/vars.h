#include <Arduino.h>

#ifndef MAIN
#define extr extern
#else
#define extr
#endif

// strategy's states
#define ATTACK 0
#define BRAKE 1
#define BOUNDS_2 2
#define KEEPER 3
#define ATTACK_LINES 4

// robot's role
#define KEEPER_ROLE  0
#define STRIKER_ROLE 1
#define SINGLE 2
#define DUAL   3

// define serial communications
#define DEBUG Serial
#define BLUETOOTH Serial1
#define PALLA_SERIAL Serial2

// define robot's velocity 
#define STRIKER_VELOCITY    80     // vel attacco 100 = ok massima nos = +10  
#define BOUNDS_VELOCITY_MIN 20      // velocità minima rampa precedente = 30
#define BOUNDS_VELOCITY_MAX 100     // velocità massima rampa abbassata 

// define constant for the BOUNDS state
#define TEMPO_FRENATA     130       // tempo di frenata prec = 100ms
#define EXTRA_EXIT_TIME    30       // tempo extra per ogni sensore attivato precedente = 50 prec = 30
#define BOUNDS_TIME_MIN    50       // tempo minimo  rientro prec = 50
#define BOUNDS_TIME_MAX   500       // tempo massimo rientro 
#define COEFF_VELOCITY_BOUNDS 1.4   // coefficente angolare rampa di velocità linee prec = 1.4
#define TIME_RAMP_BOUNDS 100        // tempo rampa velocità poi calcolo tempo uscita proporzionale prec = 100  

// porta di attacco
#define BLUE    true
#define YELLOW  false

// frequenze PWM
#define PWM_MOTORS 5000

//Pins
#define BUZZER 6 // pin buzzer
#define SW1  39  // ROLE
#define SW2  38  // ATTACK GOAL
#define SW3  33  // NOS


// utility functions to print data without overload the serial monitor
void Print(String, unsigned int);   // data_string, timer 
void Println(String, unsigned int); // print with new line character included
