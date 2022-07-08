// drive roller esc motor 
// esc davide sequenza d'avvio: 
// - write(45) (minimo) per 2 sec almeno.
// poi write(x)
#include "Arduino.h"
#include "PWMServo.h"
#include "roller/driveRoller.h"


PWMServo roller; 


// inizializza esc roller davide 
void initRoller() {
    roller.attach(PIN_ROLLER);
    roller.write(45);
}

// spegni roller 
void rollerOff() {
    roller.write(45);
}

// velocità media-bassa
void rollerOn() {
    roller.write(80); // prec = 70, 80 
}

// velocità massima 
void rollerTurbo() {
    roller.write(120);
}



