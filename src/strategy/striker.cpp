#include <Arduino.h>
#include "strategy/striker.h"
#include "data/DataSensorsCompute.h"
#include "data/DataSensorsRead.h"
#include "sensors/ballRead.h"
#include "kicker/driveKicker.h"
#include "roller/driveRoller.h"
#include "motors/motoHolon.h"
#include "sensors/lines.h"
#include "systems/degreesRemapping.h"
#include "bluetooth.h"
#include "vars.h"


/*
if ((angBall < 90) || (angBall > 270)) { // 90, 270
    centerFlag = false; 
    if(angBall < 90) *direction = (int)(angBall * COEFF_GOALIE);
    else if(angBall == 0) *direction = 0;
    else *direction = (int)(360 - ((360 - angBall) * COEFF_GOALIE)); 
*/

// flag attack goal 
bool attackGoal;

float currentOrient = 0.0; 


void initAttackGoal() {
  // init attack goal
  pinMode(SW2, INPUT); 
  if(digitalRead(SW2) == HIGH) attackGoal = BLUE;
  else attackGoal = YELLOW;
}


void attaccante(data dataRobot, int * direction) {
  
  int angBall = dataRobot.angleBallC;
  static bool centerFlag = false; 

  if ((angBall < 90) || (angBall > 270)) { // 90, 270
    centerFlag = false; 
    if(angBall < 90) {
      *direction = (int)(angBall * COEFF_GOALIE);
      //if(*direction >= 110) *direction = 110;
    }
    else if(angBall == 0) *direction = 0;
    else {
      *direction = (int)(360 - ((360 - angBall) * COEFF_GOALIE)); 
      //if(*direction <= 250)  *direction = 250; 
    } 
  }
  
  // se vedo la palla dietro al robot allora giro a destra 140°
  else if ((angBall < 230) && (angBall > 130)) { // 210, 150  offset_prec = 70
    // fix posizione 
    if(dataRobot.coordX >= 5 && centerFlag == false)  *direction = angBall + 60;
    else if(dataRobot.coordX <= -5  &&  centerFlag == false) *direction = angBall - 60; 
    else { // center
      *direction = 130; 
      //centerFlag = true; RISOLTO BUG presa palla dietro ai lati del campo 
    }
  }
  
  // se vedo la palla a destra o sinistra del robot allora vado in direzione della palla con un angolo 
  // più aperto per fare la giusta traiettoria.
  else {
    centerFlag = false; 
    if ((angBall >= 90) && (angBall <= 130))  {
      *direction = angBall + ANG_TRAIETTORIA;
    }
    else  {
      *direction = angBall - ANG_TRAIETTORIA;
    }
  } 
  
  // fix direction bug 
  *direction = abs(*direction);
  *direction = *direction % 360;  
}



void orientToGoal(struct data dataRobot,int * orient) {
  // modifica l'orientamento della bocca del robot in base all'angolo della porta di attacco.
  // se vedo la porta e l'angolo del blob è nuovo allora modifico l'orientamento, altrimenti non lo modifico.

  int porta = 0; 

  // se attacco porta blu  
  if(attackGoal == BLUE) {
    // se il dato del blob è nuovo e vedo la porta allora l'orientamento sarà verso la porta
    if(dataRobot.angBlueC != BLOB_NOT_FOUND) {
      porta = dataRobot.angBlueC;
    }
    else porta = 0; 
  }
  // se attacco porta gialla
  else { 
    // se il dato del blob è nuovo e vedo la porta allora l'orientamento sarà verso la porta,
    if(dataRobot.angYellowC != BLOB_NOT_FOUND) {
      porta = dataRobot.angYellowC;
    }
    else porta = 0; 
  }
  
  //  ORIENT con offset 
  //  if(porta > 15) porta += 10; 
  //  if(porta < 345) porta -= 10;
  //  -----------------------------

  *orient = porta;
}



// robot's strategy with kicker 
void kick_strategy(struct data dataRobot) {
  
  static unsigned long t = 0;

  if((dataRobot.coordY > 0 && dataRobot.coordY != COORD_NOT_FOUND) || dataRobot.coordY == UNDEFINED_COORD) t = millis(); 

  // se sto nella metà campo avversaria e sono orientato verso la porta di attacco  
  if((millis() -  t) < 300 && deltaAng(dataRobot.compass, dataRobot.attackAng) < 10) kicker(true);
  else kicker(false);
}



// tiro a giro se ho la palla 
/*
bool score(int compass, bool side, int *orient, int *direction, int *velocity) {
  
  static byte state = 1; 
  static unsigned long time = millis(); 
  
  switch (state)  {
    
    // vado indietro e giro lentamente 
    case (1): {
      
      rollerTurbo(); 
      
      if((millis() - time) < TIME_DIETRO) {
        if(side == true) *orient = rotate(compass, false, 30);
        else *orient = rotate(compass, true, 30); 
        *direction = fixOrient(180, compass); 
        *velocity = 110; 
      }
      else {
        *velocity = 0; 
        if(side == true) {
          *orient = rotate(compass, false, 15);
          if(compass <= 120) state = 2;  // prec = 160
        } 
        else  {
          *orient = rotate(compass, true, 15);
          if(compass >= 200) state = 2; 
        }
      }

      break;
    }

    // tiro 
    case (2): {
      rollerTurbo();

      if(side == true) {
        *orient = rotate(compass, false, VEL_SPIN);
        if(compass <= 50) {
          rollerOff(); 
          state = 0; 
          *orient = 0; 
          return true;
        }  
      }
      else {
        *orient = rotate(compass, true, VEL_SPIN);
        if(compass >= 310) {
          rollerOff();
          state = 0;
          *orient = 0;  
          return true; 
        } 
      }
    
      break;
    }

  }

  return false; 
}
*/

// inizializzo variabile currentOrient (occhio)
void initRotate(float bussola) {
  currentOrient = bussola; 
}


// ruota il robot su se stesso 
int rotate(bool sense, float Rvel) {

  if(sense == true) { // ORARIO
    currentOrient = currentOrient + Rvel;
  }
  else { // anti-orario
    currentOrient = currentOrient - Rvel;
  }

  if(currentOrient < 0)    currentOrient += 360; 
  if(currentOrient > 360)  currentOrient -= 360;   

  return (int)currentOrient;
}






/*
FUNZIONE ATTACCANTE CON DISTANZA
void attaccante(data dataRobot, int * direction) {
  
  int angBall = dataRobot.angleBallC;
  static bool centerFlag = false;  

  // CORREZIONE ANGOLO PALLA SOTTRAGGO I GRADI (solo rick)  
  if((angBall > 300) || (angBall < 10)) {
    angBall = angBall - 10;
    if(angBall < 0) angBall += 360;
  }
  // -------------


  // TEST coeff_goalie con distanza 
  float coeff_goalie; 
  if(dataRobot.distanceBallC < 117) {
    coeff_goalie = COEFF_CLOSE; 
  }
  else {
    coeff_goalie = COEFF_FAR;
  }
  // --------------

    
  if(angBall <= 180) *direction = (int)(angBall * coeff_goalie);
  else if(angBall > 180) *direction = (int)(360 - ((360 - angBall) * coeff_goalie)); 
  else if(angBall == 0)  *direction = 0; 

  
  // palla dietro con posizione 
  else if ((angBall < 210) && (angBall > 150) && ((dataRobot.ballDist_thresholds == CLOSE) || (dataRobot.ballDist_thresholds == VERY_CLOSE))) {
    // fix posizione 
    if(dataRobot.coordX >= 5 && centerFlag == false)  *direction = angBall + 70;
    else if(dataRobot.coordX <= -5  &&  centerFlag == false) *direction = angBall - 70; 
    else { // center
      *direction = 130; 
      centerFlag = true; 
    }
  }

  // reset centerFlag
  if ((angBall < 90) || (angBall > 270))  centerFlag = false; 
  
  // fix direction bug 
  *direction = abs(*direction);
  *direction = *direction % 360;  
}


void attaccante(data dataRobot, int * direction) {
  
  int angBall = dataRobot.angleBallC;
  static bool centerFlag = false; 

  // CORREZIONE ANGOLO PALLA SOTTRAGGO I GRADI (solo rick)  
  if((angBall > 300) || (angBall < 10)) {
    angBall = angBall - 10;
    if(angBall < 0) angBall += 360;
  }
  // -------------


  // TEST coeff_goali con distanza 
  float coeff_goalie; 
  if(dataRobot.ballDist_thresholds == VERY_CLOSE) coeff_goalie = 2.5; 
  else if(dataRobot.ballDist_thresholds == CLOSE) coeff_goalie = 2.5;
  else coeff_goalie = 1.2; 
  // --------------


  if ((angBall < 90) || (angBall > 270)) {
    centerFlag = false; 
    if(angBall < 90) *direction = (int)(angBall * coeff_goalie);
    else if(angBall == 0) *direction = 0;
    else *direction = (int)(360 - ((360 - angBall) * coeff_goalie)); 
  }

  
  // se vedo la palla dietro al robot allora giro a destra 140°
  else if ((angBall < 210) && (angBall > 150)) {
    // fix posizione 
    if(dataRobot.coordX >= 5 && centerFlag == false)  *direction = angBall + 70;
    else if(dataRobot.coordX <= -5  &&  centerFlag == false) *direction = angBall - 70; 
    else { // center
      *direction = 130; 
      centerFlag = true; 
    }
  }
  
  // se vedo la palla a destra o sinistra del robot allora vado in direzione della palla con un angolo 
  // più aperto per fare la giusta traiettoria.
  else {
    centerFlag = false; 
    if ((angBall >= 90) && (angBall <= 150))  {
      *direction = angBall + ANG_TRAIETTORIA;
    }
    else  {
      *direction = angBall - ANG_TRAIETTORIA;
    }
  }


  // fix direction bug 
  *direction = abs(*direction);
  *direction = *direction % 360;  
}

*/