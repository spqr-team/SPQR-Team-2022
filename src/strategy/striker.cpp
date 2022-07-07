#include <Arduino.h>
#include "strategy/striker.h"
#include "data/DataSensorsCompute.h"
#include "data/DataSensorsRead.h"
#include "sensors/ballRead.h"
#include "kicker/driveKicker.h"
#include "bluetooth.h"
#include "vars.h"



// flag attack goal 
bool attackGoal;


void initAttackGoal() {
  // init attack goal
  pinMode(SW2, INPUT); 
  if(digitalRead(SW2) == HIGH) attackGoal = BLUE;
  else attackGoal = YELLOW;
}


void attaccante(data dataRobot, int * direction) {
  
  int angBall = dataRobot.angleBallC;
  static bool centerFlag = false; 


  if ((angBall < 90) || (angBall > 270)) {
    centerFlag = false; 
    if(angBall < 90) *direction = (int)(angBall * COEFF_GOALIE);
    else if(angBall == 0) *direction = 0;
    else *direction = (int)(360 - ((360 - angBall) * COEFF_GOALIE)); 
  }
  
  // se vedo la palla dietro al robot allora giro a destra 140°
  else if ((angBall < 210) && (angBall > 150)) {
       *direction = 130; 
      centerFlag = true; 
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
 
  *orient = porta;
}



// robot's strategy with kicker 
void kick_strategy(struct data dataRobot) {
  
  static unsigned long t = 0;

  if(dataRobot.coordY > 10) t = millis(); 

  if((millis() -  t) < 300) kicker(true);
  else kicker(false);

}


