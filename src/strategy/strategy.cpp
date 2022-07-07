// definizione funzioni strategia, qui decido che deve fare il robot in base ai dati elaborati.

#include <Arduino.h>
#include "data/DataSensorsCompute.h"
#include "systems/degreesRemapping.h"
#include "systems/position.h"
#include "motors/motors.h"
#include "strategy/keeper.h"
#include "motors/motoHolon.h"
#include "strategy/striker.h"
#include "sensors/lines.h"
#include "sensors/presa_palla.h"
#include "sensors/ballRead.h"
#include "roller/driveRoller.h"
#include "kicker/driveKicker.h"
#include "vars.h"
#include "bluetooth.h"



int computeStrategy(struct data * dataRobot, int * direction, int * velocity, int * orient) {

    static unsigned long t = 0;         // utility timer
    static unsigned int ExitTime = 0;   // tempo di uscita che viene incrementato
    static unsigned long tRamp = 0;     // tempo velocity ramp lines

    static int angleBprec = 0;          // angle ball prec 
    static unsigned long boundsT = 0; 
    static bool backToGoal = false;     // flag ritorno in porta  
    static int exitDir = 0; 

    static int STATE = KEEPER;          // current  STATE
    static int STATE_prec = STATE;      // previous STATE

    static bool ExitTime_computed = false;    // flag computed exit time 
    static bool fix = false;
    static bool center = false;
    static bool OUT_lines = false; 
    static bool compute_ramp = false;        // flag velocity ramp 

    static bool lock = false;    



    // ------ CAMBIO RUOLI ATTACCANTE-PORTIERE  (ritornp in porta)
    /*
    // portiere
    if(dataRobot->role == KEEPER_ROLE) {
      if(STATE == ATTACK) {
        STATE = KEEPER; 
        backToGoal = false; 
      }
    }  
    else if(dataRobot->role == STRIKER_ROLE) { // attaccante
      if(STATE != BOUNDS && STATE != ATTACK) STATE = ATTACK; 
    }
    */
    // ------ TEST -------


    
    switch(STATE) {
      
      case(ATTACK): {

        if ((*dataRobot).FLAG_lines) {
          STATE = BOUNDS_2;
          STATE_prec = ATTACK; 
          t = millis();
          angleBprec = dataRobot->angleBallC;
          lock = true; 
        }
        else {

          orientToGoal((*dataRobot), orient);
          attaccante((*dataRobot), direction);
          *velocity = STRIKER_VELOCITY;
          kicker(false); // (no kick) charge the capacitor 

          // compute velocity after bounds 
          if((millis() - boundsT) < 500) {
            if(lock == true && deltaAng(angleBprec, dataRobot->angleBallC) < 30) *velocity = 50; 
            else lock = false; 
          }

          // se non vedi la palla vai al centro  
          if(dataRobot->distanceBallC > 240) {
            gotoCenter(*dataRobot, direction, velocity, orient);
          }  

        }               
        break;
      }
      

      // BOUNDS esco fino a che vedo la linea -----
      case(BOUNDS_2): {

        // se uscito completamente 
        if(dataRobot->lineSensors_activated_number > 6 && OUT_lines == false) {
          ExitTime += 300;
          OUT_lines = true; 
        }
        
        if((millis() - t) < ExitTime) {
          // direction
          *direction = linesToCenter(*dataRobot);  
          *direction = fixOrient(*direction, dataRobot->compass);
          // velocity
          if(compute_ramp == false) { 
            tRamp = millis();
            compute_ramp = true; 
          } 
          *velocity = linesVelRamp(tRamp); 
        }
        else {
          if(dataRobot->lineSensors_current > 0) ExitTime += 40; 
          else {  // RESET 
            dataRobot->lineSensors_prec = 0;        // reset sensori prec
            ExitTime = 0;                           // reset tempo di rientro 
            dataRobot->resetActivatedSens = true;   // reset activated sensors
            ExitTime_computed = false;              // reset flag calcolo exit time
            OUT_lines = false;                      // reset flag out
            STATE = STATE_prec;                     // gioco
            compute_ramp = false;                   // reset velocity ramp 
            // TEST timer 
            boundsT = millis();
            t = millis();
            noTone(BUZZER);
          }
        }
        break; 
      }


      case(KEEPER): {
        // keeper dir and vel 
        // se vedo la porta di difesas
        if((*dataRobot).defenseAng != BLOB_NOT_FOUND) {
          // se sto lontano dalla porta di difesa 
          if((dataRobot->defenseArea < AREA_DEFENSE) && (backToGoal == false))  {
            Keeper_compute(dataRobot, direction, velocity, orient, true, false);
            if ((*dataRobot).FLAG_lines) { // lines
              STATE = BOUNDS_2;
              STATE_prec = KEEPER;
              t = millis();
            }
          }
          else {  
            
            backToGoal = true;

            // ritorno in porta se lontano dall'area di difesa 
            if(dataRobot->defenseArea < AREA_DEFENSE) {
              Keeper_compute(dataRobot, direction, velocity, orient, true, false);
              if ((*dataRobot).FLAG_lines) { 
                STATE = BOUNDS_2;
                STATE_prec = KEEPER;
                t = millis();
              }
            }
            else {
              // vai al centro porta se non vedi la palla altrimenti fai portiere
              if(dataRobot->ballDist_thresholds == NOT_SEE) Keeper_compute(dataRobot, direction, velocity, orient, false, true);
              else Keeper_compute(dataRobot, direction, velocity, orient, false, false);
            }
          }
        }  
        else {  // se non vedo la porta di difesa vai a (0,-10)
          *direction = GoToCoords(0, -10, dataRobot->coordX, dataRobot->coordY);
          *velocity = 60; 
          *orient = 0;
          if ((*dataRobot).FLAG_lines) { // lines
            STATE = BOUNDS_2;
            STATE_prec = KEEPER;
            t = millis();
          }
        }
        
        break;
      }
  
    }  
  
  //DEBUG 
  //BTdebug_addData("direction " + String(*direction));
  //BTdebug_addData("velocity " + String(*velocity));
  //BTdebug_addData("distanceBall " + String(dataRobot->ballDist_thresholds));

  // TEST a casa
  //if(dataRobot->lineSensors_current > 0)  digitalWrite(BUZZER, HIGH);
  //else digitalWrite(BUZZER, LOW);

  // TEST
  /* 
  Serial.println("X: " + String(dataRobot->coordX));
  Serial.println("Y: " + String(dataRobot->coordY)); 
  Serial.println("dir: " + String(linesToCenter(*dataRobot)));
  Serial.println("direction " + String(*direction));
  */
 
  //lineSensorsTest(); 
  //readAllSensors();
  //presaPalla(); 
  //delay(5);

  //Serial.println("ball: " + String(dataRobot->angleBallC));
  //Serial.println("dist: " + String(dataRobot->ballDist_thresholds));
  //Serila.println("zone: " + String());
  //Serial.println("direction: " + String(*direction));

  //readAtmega32u4();
  //Serial.println("angle: " + String(dataRobot->angleBallC));
  //Serial.println("dist: " + String(dataRobot->distanceBallC));
  //delay(5);
  
  //testMot(); 

  //Serial.println("state: " + String(STATE));
  //Serial.println("bno: " + String(dataRobot->compass));

  return STATE;
}



