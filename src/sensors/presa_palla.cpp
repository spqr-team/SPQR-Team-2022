// sensori di presa palla 
#include "Arduino.h"
#include "sensors/presa_palla.h"



// true se ho la palla in bocca 
// false se non ho la palla 
bool presaPalla() {

    int value = analogRead(PIN_PRESA_PALLA);
    static bool ballPresence = false; 
    static unsigned long time = millis();  
    
    // ---- TEST ----
    Serial.print("400 ");
    Serial.println(value);


    // no ball presence
    if(ballPresence == false) {
        if(value >= SOGLIA_PRESA_PALLA) time = millis(); 
        if((millis() - time) > 100) ballPresence = true; 
    }
    else { // ball in mounth
        if(value <= SOGLIA_PRESA_PALLA) time = millis(); 
        if((millis() - time) > 100) ballPresence = false;
    }

    return !ballPresence;
}

// test presaPalla
void test_presaPalla() {
    Serial.println(analogRead(PIN_PRESA_PALLA));
}


