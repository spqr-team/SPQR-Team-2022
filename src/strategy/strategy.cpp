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

elapsedMillis timerBallOUT = 0; 
elapsedMillis timerPresaPalla = 0;
elapsedMillis TimerLines = 0; 
elapsedMillis timerFlag_orient = 0;  



int computeStrategy(struct data * dataRobot, int * direction, int * velocity, int * orient) {

    static unsigned long t = 0;         // utility timer
    static unsigned int ExitTime = 0;   // tempo di uscita che viene incrementato
    static unsigned long tRamp = 0;     // tempo velocity ramp lines

    static int angleBprec = 0;          // angle ball prec 
    static unsigned long boundsT = 0;   // time after state BOUNDS
    static unsigned long attack_linesT = 0; // timer state ATTACK_LINES
    static unsigned long Tlines = millis(); // timer visione linea ATTACK_LINES
    static bool backToGoal = false;     // flag ritorno in porta  
    static int exitDir = 0; 

    static int STATE = ATTACK;          // current  STATE
    static int STATE_prec = STATE;      // previous STATE

    static bool ExitTime_computed = false;    // flag computed exit time 
    static bool fix = false;
    static bool center = false;
    static bool OUT_lines = false; 
    static bool compute_ramp = false;        // flag velocity ramp 
    static bool flag_attackLines = false;    // flag attack on lines  

    static bool lock = false; 
    byte Stotal = dataRobot->lineSensors_current;

    static bool areaFlag = false; 
    static unsigned long timerArea = millis();
    static bool stop = false; 
    static bool ballOUTDX = false; 
    static bool ballOUTSX = false; 
    static bool flag_orient = false; 


    // TIMER
    // timer presa palla 
    if(presaPalla(*dataRobot))  timerPresaPalla = 0;
    // timer linea
    if(dataRobot->lineSensors_current > 0)  TimerLines = 0; 


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

          // FIX vai verso la palla quando sei sulla linea dell'area di difesa 
          // ------------------------------------------
          // Stotal = dataRobot->lineSensors_current; 
          if(dataRobot->lineSensors_activated_number <= 2 && SE) {
            areaFlag = true; 
            timerArea = millis(); 
          }
          else if (dataRobot->lineSensors_activated_number > 2)  {
            areaFlag = false; 
            timerArea = 0; 
          }
          else {
            if((millis() - timerArea) < 500)  areaFlag = true;  
            else areaFlag = false; 
          }
          if (areaFlag == true) {
            *direction = dataRobot->AngleBallC_absolute;
            *direction = fixOrient(*direction, dataRobot->compass);
          }
          else {
            STATE = BOUNDS_2;
            STATE_prec = ATTACK;
          }
          // ----------------------------

          // TEST condizione palla fuori dal campo 
          if(dataRobot->lineSensors_activated_number == 1 && (EE || OE))  {
            if(EE && dataRobot->angleBallC > 30  && dataRobot->angleBallC < 150) ballOUTDX = true; 
            if(OE && dataRobot->angleBallC > 210 && dataRobot->angleBallC < 330) ballOUTSX = true; 
          }  

          
         

          t = millis();
        }
        else {

          // se non vedi la palla vai al centro
          if(dataRobot->distanceBallC > 240) {
            gotoCenter(*dataRobot, direction, velocity, orient); 
            rollerOff(); 
            kicker(false);
            *velocity = 40; 
            break;
          }
          else {
            
            // roller 
            if(dataRobot->angleBallC >= 300 || dataRobot->angleBallC <= 60) rollerOn(); 
            else rollerOff(); 

            // timer flag_orient
            if(flag_orient == true) {
              timerFlag_orient = 0; 
              flag_orient = false; 
            }

            // attacco con palla 
            if(presaPalla((*dataRobot))) {
              orientToGoal((*dataRobot), orient);
              attaccante((*dataRobot), direction);
              *velocity = STRIKER_VELOCITY;
              if(timerFlag_orient > 400)  kick_strategy(*dataRobot); 
            }
            else { // attacco senza palla 
              orientToGoal((*dataRobot), orient);
              attaccante((*dataRobot), direction);
              *velocity = STRIKER_VELOCITY;
              kicker(false); // (no kick) charge the capacitor 
              t = millis(); 
            }
            
            // reset 
            if(dataRobot->lineSensors_activated_number > 1 || dataRobot->angleBallC > 150 || dataRobot->angleBallC < 30)  ballOUTDX = false; 
            if(dataRobot->lineSensors_activated_number > 1 || dataRobot->angleBallC > 330 || dataRobot->angleBallC < 210) ballOUTSX = false;
            
            if(ballOUTDX || ballOUTSX)  *velocity = 40;

            if(ballOUTDX || ballOUTSX)  {
              *velocity = 40;
              if(timerBallOUT > 1500) {
                // segui la palla 
                *direction = dataRobot->AngleBallC_absolute;
                *direction = fixOrient(*direction, dataRobot->compass);
                if(timerBallOUT > 2000)  {
                  STATE = ATTACK_LINES; 
                  flag_orient = false; 
                  dataRobot->resetActivatedSens = true;
                  timerBallOUT = 0;  
                  ballOUTDX = false; 
                  ballOUTSX = false;  
                }
              }
            }
            else timerBallOUT = 0;
            
          } 

        }              
        break;
      }
      

      // BOUNDS esco fino a che vedo la linea -----
      case(BOUNDS_2): {
        
        //tone(BUZZER, 500);

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
          if(flag_attackLines == true)  *velocity = 60; // esco piano se attacco sule linee
        }
        else {
          if(dataRobot->lineSensors_current > 0) ExitTime += 40; // prec = 40  
          else {  // RESET 
            dataRobot->lineSensors_prec = 0;        // reset sensori prec
            ExitTime = 0;                           // reset tempo di rientro 
            dataRobot->resetActivatedSens = true;   // reset activated sensors
            ExitTime_computed = false;              // reset flag calcolo exit time
            OUT_lines = false;                      // reset flag out
            STATE = STATE_prec;                     // gioco
            STATE_prec = BOUNDS_2;                  // state prec = BOUNDS
            compute_ramp = false;                   // reset velocity ramp 
            flag_attackLines = false;               // reset flaqg attack on lines 
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
        // se vedo la porta di difesa
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


      // TEST ATTACCO SULLA LINEA ------------
      case(ATTACK_LINES): {

        rollerTurbo(); 

        // RESET allo stato ATTACK se non vedo la linea da un po di tempo o ho la palla 
        /*
        if(TimerLines > 500)  {
          STATE = ATTACK; 
          STATE_prec = ATTACK_LINES;
        }
        */
        // se si sono attivati più di 6 sensori (robot uscito più di metà) 
        // reagisco alle linee state = BOUNDS_2
        if(dataRobot->lineSensors_activated_number > 7) {
          STATE = ATTACK;  
          flag_attackLines = true;  
        }
        else if(TimerLines > 1000 && timerPresaPalla > 1000) STATE = ATTACK; // reset se non vedo linea da 500ms 
        else if(timerPresaPalla < 300) {
          *direction = linesToCenter(*dataRobot);  
          *direction = fixOrient(*direction, dataRobot->compass);
          if(deltaAng(dataRobot->compass, dataRobot->attackAng) > 10) {
            if(dataRobot->coordX >= 0) *orient = rotate(false, 0.07);
            else *orient = rotate(true, 0.07);
          } 
          else STATE = ATTACK; 
          *velocity = 40; 
          if(TimerLines > 100)  STATE = ATTACK;  
          flag_orient = true; 
        }
        else {   
          *orient = dataRobot->AngleBallC_absolute;
          *direction = dataRobot->AngleBallC_absolute;
          *direction = fixOrient(*direction, dataRobot->compass);
          initRotate(dataRobot->compass); 
          *velocity = 40; 
        }

        // timer ATTACK_LINES
        attack_linesT = millis();

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
  //Serial.println("X: " + String(dataRobot->coordX));
  //Serial.println("Y: " + String(dataRobot->coordY)); 
  //delay(5);
  //Serial.println("dir: " + String(linesToCenter(*dataRobot)));
  //Serial.println("direction " + String(*direction));
 
  //lineSensorsTest(); 
  //readAllSensors();
  //delay(5);
  //presaPalla(); 

  //Serial.println("ball: " + String(dataRobot->angleBallC));
  //Serial.println("dist: " + String(dataRobot->distanceBallC));
  //Serila.println("zone: " + String());
  //Serial.println("direction: " + String(*direction));

  //readAtmega32u4();
  //Serial.println("angleB: " + String(dataRobot->angleBallC));
  //Serial.println("compass: " + String(dataRobot->compass));
  //Serial.println("dist: " + String(dataRobot->distanceBallC));
  //Serial.println("state: " + String(STATE));
  //Serial.println("bno: " + String(dataRobot->compass));
  //delay(5);
  
  //testMot();

  //if(presaPalla(*dataRobot))  tone(BUZZER, 600);
  //else noTone(BUZZER); 


  return STATE;
}



