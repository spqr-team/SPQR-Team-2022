// openMV h7 camera
#include <Arduino.h>
#include "sensors\camera.h"
#include "bluetooth.h"



bool readCAM(int *blueGoal, int *yellowGoal, int *areaBlue, int *areaYellow, int orient) {
   // FUNZIONA, ho simulato la funziona è da testare sul robot.
   // ritorna gli ultimi angoli delle porte trasmessi dalla CAM:  Y+ang+y, B+ang+b
   // gli angoli tengono conto dell'orientamento del robot.
   // ritorna true se ho letto nuovi dati dal buffer seriale altrimenti false
   // se non vede il blob ritorna BLOB_NOT_FOUND
   // orient = orientamento attuale del robot, per scalare gli angoli delle porte.

   char data;
   long angGoal  = 0;
   long areaGoal = 0; 
   bool startB = false; 
   bool endB = true;   
   bool newData = false;
   

    // finchè ci sono dati del buffer seriale  
    while (Serial3.available()) {
      // leggo un byte 
      data = Serial3.read();
      
      // start pacchetto
      if (((data == 'Y') || (data == 'B'))  &&  (endB == true)  &&  (startB == false)) {      
        startB = true;
        endB = false; 
        angGoal = Serial3.parseInt();
        Serial3.read();
        areaGoal = Serial3.parseInt();
      }
      
      // end pacchetto
      else if (((data == 'y') || (data == 'b'))  &&  (startB == true)  &&  (endB == false)) {   
        startB = false;
        endB = true; 
        // YELLOW
        if(data == 'y')   {
          // flag nuovi dati ricevuti
          newData = true; 
          // se non vedo il blob
          if(angGoal == BLOB_NOT_FOUND) {
            // se non ho visto il blob per un tempo superiore a TIMER_BLOB_NOT_FOUND
            *yellowGoal = BLOB_NOT_FOUND;
            *areaYellow = 0;
          }
          else {   // se vedo il blob aggiorno l'angolo
            *yellowGoal =  (int)(angGoal + orient) % 360;
            *areaYellow =  areaGoal;
          }
        }
        // BLUE
        if(data == 'b')   { 
          // flag nuovi dati ricevuti
          newData = true;  
          // se non vedo il blob
          if(angGoal == BLOB_NOT_FOUND) { 
            *blueGoal = BLOB_NOT_FOUND;
            *areaBlue = 0; 
          }
          else {  // se vedo il blob aggiorno l'angolo
            *blueGoal = (int)(angGoal + orient) % 360;
            *areaBlue = areaGoal;
          } 
        }
      }
    }

    return newData;
}




