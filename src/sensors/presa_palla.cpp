// sensori di presa palla
// PRESA PALLA che funziona configurazione:  
// con canalina, condensatore 1uF, filtro complementare: 0.01, condizioni palla davanti e vicina
// soglia = valore senza palla - 30 
// funziona abbastanza bene 

#include "Arduino.h"
#include "sensors/presa_palla.h"
#include "data/DataSensorsCompute.h"
#include "data/DataSensorsRead.h"






// true se ho la palla in bocca 
// false se non ho la palla 
bool presaPalla(struct data dataRobot) {

    int value = analogRead(PIN_PRESA_PALLA);
    static bool ballPresence = false; 
    static unsigned long time = millis();
    static unsigned long timeConditions = millis();  
    
    // ---- TEST ----
    //Serial.println(value);

    // no ball presence
    if(ballPresence == false) {
        if(value >= SOGLIA_PRESA_PALLA) time = millis(); 
        if((millis() - time) > 100) ballPresence = true; 
    }
    else { // ball in mounth
        if(value <= SOGLIA_PRESA_PALLA) time = millis(); 
        if((millis() - time) > 200) ballPresence = false;
    }

    // FIX se vedo la palla in bocca e davanti a me e non troppo lontana: ho la palla 
    if(ballPresence == true && (dataRobot.angleBallC > 335 || dataRobot.angleBallC < 25) && dataRobot.distanceBallC < 200)  ballPresence = true;
    else ballPresence = false; 

    return ballPresence;
}


// test presaPalla
void test_presaPalla() {
    Serial.println(analogRead(PIN_PRESA_PALLA));
}


