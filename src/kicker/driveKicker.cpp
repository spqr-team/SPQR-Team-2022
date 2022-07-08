#include "Arduino.h"
#include "kicker/driveKicker.h"

elapsedMillis timerKick = 0;

// kick the ball 
bool kicker(bool kick) { 
    // spara ogni DELAY_KICK, va chiamata ad ogni loop in modo che il condensatore
    // si carichi in tempo (per la durata del segnale basso)

    static unsigned long time = millis(); 
    static bool kick_flag = false; 

    // reset 
    if((millis() - time) > DURATION_KICK && kick_flag == true) kick_flag = false;

    if((kick == true && timerKick > DELAY_KICK) || kick_flag == true)  {
        digitalWrite(PIN_KICKER, HIGH); // kick with the solenoid
        timerKick = 0; 
        if(kick_flag == false)  time = millis();
        kick_flag = true; 
        return true; 
    }
    else {
        digitalWrite(PIN_KICKER, LOW); // charge the capacitor
        return false; 
    }

}