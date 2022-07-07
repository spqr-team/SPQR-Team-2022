#include <Arduino.h>
#include "data/DataSensorsCompute.h"
#include "data/DataSensorsRead.h"
#include "sensors/ballRead.h"
#include "sensors/lines.h"
#include "sensors/camera.h"
#include "systems/position.h"
#include "strategy/keeper.h"
#include "bluetooth.h"




void computeDataSensors(struct inputData inputDataRobot,  struct data * dataRobot)  {
    // per ora non eseguo operazioni sui dati (integrazione sistemi), work in progess

    // IMU
    dataRobot->compass = inputDataRobot.compass;


    // BALL: angle and distance
    dataRobot->angleBallC = inputDataRobot.angleBall;
    dataRobot->angleBallC = filterBallAngle(dataRobot->angleBallC);
    dataRobot->AngleBallC_absolute = (int)(inputDataRobot.angleBall + inputDataRobot.compass) % 360;
    // distance
    dataRobot->distanceBallC = inputDataRobot.distanceBall;
    dataRobot->ballDist_thresholds = distanceBall_threshold(dataRobot->distanceBallC);


    // LINES: current = sensori attivi, prec = sensori precedenti attivi, total = sensori attivati totali
    dataRobot->lineSensors_last = (*dataRobot).lineSensors_current; // update last lineSensors activated
    dataRobot->lineSensors_current = readLines_current();
    dataRobot->lineSensors_activated_number = LineSensorsActivated_number((*dataRobot).lineSensors_current, (*dataRobot).resetActivatedSens);
    dataRobot->lineSensors_activated_byte = LineSensorsActivated_byte((*dataRobot).lineSensors_current, (*dataRobot).resetActivatedSens);
    dataRobot->lineSensors_total = readLines_total((*dataRobot).lineSensors_prec, (*dataRobot).lineSensors_current);
    dataRobot->lineSensors_prec =  (*dataRobot).lineSensors_total;
    dataRobot->resetActivatedSens = false;   // FIX BUG: reset flag ActivatedSensors
    // if the robot sees bounds, compute exit angle.
    if(((*dataRobot).lineSensors_total) > 0) {
        dataRobot->FLAG_lines = true;
        dataRobot->exitAngLines = readAngLines((*dataRobot).lineSensors_total, inputDataRobot.compass);
    }
    else dataRobot->FLAG_lines = false;



    // CAM: 
    // se non vedo il blob allora assegno l'angolo del blob precedente
    // dopo il tempo TIMER_BLOB_NOT_FOUND in cui non vedo l'angolo, il robot giocherà senza l'angolo del blob.
    // se ci sono nuovi dati dalla CAM aggiorno gli angoli delle porte 
    if(inputDataRobot.newDataCAM) {
        // flag valori vecchi disattivati per indicare che gli angoli sono attuali
        dataRobot->old_angBlue = false;
        dataRobot->old_angYellow = false;

        // se non vedo blob assegno valore precedente e scrivo flag che non vedo il blob 
        // blue
        if(inputDataRobot.angBlu == BLOB_NOT_FOUND) {
            if((millis() - (*dataRobot).timerBlue)  <  TIMER_BLOB_NOT_FOUND) {
                dataRobot->angBlueC = (*dataRobot).angBlue_prec;  // previous angle
                dataRobot->areaBlueC = dataRobot->areaBlue_prec;  // previous area 
                dataRobot->old_angBlue = true;                    // flag old angle = true
                dataRobot->newBLOB_blue = false;                  // flag new angle = false
            }
            else {
                dataRobot->angBlueC = inputDataRobot.angBlu;   
                dataRobot->areaBlueC = inputDataRobot.areaB;
            }
        }
        else  { // se vedo il blob
            dataRobot->angBlueC = inputDataRobot.angBlu;      // update new angle
            dataRobot->areaBlueC = inputDataRobot.areaB;      // update area
            dataRobot->areaBlue_prec = inputDataRobot.areaB;    
            dataRobot->angBlue_prec = inputDataRobot.angBlu;  // reset previous blob
            dataRobot->newBLOB_blue = true;                   // flag new blob found
            dataRobot->old_angBlue = false;                   // flag old blob
            dataRobot->timerBlue = millis();                  // reset timer blob blue
        }

        // yellow
        if(inputDataRobot.angYellow == BLOB_NOT_FOUND) {
            if((millis() - (*dataRobot).timerYellow)  <  TIMER_BLOB_NOT_FOUND) {
                dataRobot->angYellowC = (*dataRobot).angYellow_prec;    // previous angle
                dataRobot->areaYellowC = dataRobot->areaYellow_prec;    // previous area
                dataRobot->old_angYellow  = true;                       // flag old angle = true
                dataRobot->newBLOB_yellow = false;                      // flag new angle = false
            }
            else {
                // BLOB_NOT_FOUND timer expires
                dataRobot->angYellowC = inputDataRobot.angYellow;      
                dataRobot->areaYellowC = inputDataRobot.areaY;
            }
        } 
        else  { // se vedo il blob
            dataRobot->angYellowC = inputDataRobot.angYellow;      // update new angle
            dataRobot->areaYellowC = inputDataRobot.areaY;         // update area 
            dataRobot->areaYellow_prec = inputDataRobot.areaY;      
            dataRobot->angYellow_prec = inputDataRobot.angYellow;  // reset angBlob precedente
            dataRobot->newBLOB_yellow = true;                      // flag new blob found
            dataRobot->old_angYellow = false;                      // flag old blob
            dataRobot->timerYellow = millis();                     // reset timer yellow
        }
    }


    // POSITION:
    int coordX = COORD_NOT_FOUND;
    int coordY = COORD_NOT_FOUND;
    int angB_position = dataRobot->angBlueC;
    int angY_position = dataRobot->angYellowC; 
    static unsigned long timerPosition = 0;
    if(angB_position != BLOB_NOT_FOUND && angY_position != BLOB_NOT_FOUND) {
        angB_position = filterAngCAM_blue(angB_position);
        angY_position = filterAngCAM_yellow(angY_position); 
    }

    computeCoords(angB_position, angY_position, dataRobot->attackGoal, &coordX, &coordY);    
    
    // se non ho la posizione considero la precedente 
    if(coordX == COORD_NOT_FOUND && coordY == COORD_NOT_FOUND) {
        if(dataRobot->coordX_prec != COORD_NOT_FOUND && dataRobot->coordY_prec != COORD_NOT_FOUND) {
            dataRobot->coordX = dataRobot->coordX_prec;
            dataRobot->coordY = dataRobot->coordY_prec;
            // position data timer
            if(dataRobot->prevPosition == false) {
                timerPosition = millis(); 
            }
            else dataRobot->timePosition_data = millis() - timerPosition;
            dataRobot->prevPosition = true; 
            dataRobot->positionAvailable = true;
        } 
        else {
            dataRobot->coordX = COORD_NOT_FOUND;
            dataRobot->coordY = COORD_NOT_FOUND;
            dataRobot->positionAvailable = false;
        }
    }
    else {
        dataRobot->coordX = coordX; 
        dataRobot->coordY = coordY;
        // reset previos coordinates
        dataRobot->coordX_prec = coordX; 
        dataRobot->coordY_prec = coordY;
        dataRobot->prevPosition = false;
        dataRobot->positionAvailable = true;
        // reset position timer 
        dataRobot->timePosition_data = 0;
    }

 
    // compute enemy's goal and defense goal 
    if(digitalRead(SW2) == HIGH) { 
        dataRobot->attackGoal = BLUE;
        // defense 
        dataRobot->defenseAng  = dataRobot->angYellowC;
        dataRobot->defenseArea = dataRobot->areaYellowC;
        // attack 
        dataRobot->attackAng  = dataRobot->angBlueC;
        dataRobot->attackArea = dataRobot->areaBlueC;
    }
    else {
        dataRobot->attackGoal = YELLOW;
        // defense 
        dataRobot->defenseAng  = dataRobot->angBlueC;
        dataRobot->defenseArea = dataRobot->areaBlueC;
        // attack 
        dataRobot->attackAng  = dataRobot->angYellowC;
        dataRobot->attackArea = dataRobot->areaYellowC;
    }


    // compute role 
    // se l'altro robot è fuori allora diventa portiere 
    if(BTisConnected() == false) {
        dataRobot->role = KEEPER_ROLE;
        dataRobot->mode = SINGLE; // mod: un solo robot 
    }
    else { // se sono connessi 
        dataRobot->mode = DUAL; // mod: entrambi i robot 
        if(digitalRead(SW1) == LOW)  dataRobot->role = KEEPER_ROLE; 
        else  dataRobot->role = STRIKER_ROLE;
    }

    // compute keeper values 
    init_keeper(dataRobot->attackGoal);

    
    //  compute NOS strategy
    if(digitalRead(SW3) == HIGH) dataRobot->NOS = true; 
    else dataRobot->NOS = false; 


    // -----TEST----------
    // test presa palla
    //Serial.print(" 500 ");
    //Serial.println(String(analogRead(A13)));
    //delay(5);

    
    // test camera
    //Serial.println("blobYellow " + String(dataRobot->angYellowC));
    //Serial.println("blobBlue " + String(dataRobot->angBlueC));
    //Serial.println("areaY: " + String(dataRobot->areaYellowC));
    //Serial.println("areaB: " + String(dataRobot->areaBlueC));
    

    // test ball
    /*
    Serial.println("angBall: " + String(dataRobot->angleBallC));
    Serial.println("distanceBall: " + String(dataRobot->distanceBallC));
    */
   
    //BTdebug_addData("blobBlue " + String(dataRobot->angBlueC));
    //BTdebug_addData("blobYellow " + String(dataRobot->angYellowC));
    //BTdebug_addData("areaY " + String(dataRobot->areaYellowC));
    //BTdebug_addData("areaB " + String(dataRobot->areaBlueC));

    //BTdebug_addData("coordX " + String(dataRobot->coordX));
    //BTdebug_addData("coordY " + String(dataRobot->coordY));
    //BTdebug_addData("coordX_prec " + String(dataRobot->coordX_prec));
    //BTdebug_addData("coordY_prec " + String(dataRobot->coordY_prec));
}
