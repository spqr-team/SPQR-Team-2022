// position system 
#include <Arduino.h>
#include "sensors/camera.h"
#include "systems/degreesRemapping.h"
#include "systems/position.h"
#include "sensors/lines.h"
#include "data/DataSensorsCompute.h"
#include "systems/degreesRemapping.h"
#include "motors/motoHolon.h"
#include "bluetooth.h"
#include "vars.h"



// calcola le coordinate cartesiane del robot nel campo in base agli angoli delle porte
// le coordinate sono riferite ad un piano cartesiano ideale -50+50 con origine al centro del campo.
void computeCoords(int angB, int angY, bool attack, int * coordX, int * coordY) {

  // se vedo entrambe le porte
  if ((angB != BLOB_NOT_FOUND) && (angY != BLOB_NOT_FOUND)) {

    // variabili con angoli goniometrici porte
    float attackGoal  = 0.0;   // attacco
    float defenseGoal = 0.0;   // difesa

    // converto i gradi del robot nei gradi goniometrici
    // se attacco porta blu
    if (attack == BLUE) {
      attackGoal  = ToGoniometricAngles((float) angB);    // attacco
      defenseGoal = ToGoniometricAngles((float) angY);    // difesa
    }
    else {  // attacco porta gialla
      attackGoal  = ToGoniometricAngles((float) angY);    // attacco
      defenseGoal = ToGoniometricAngles((float) angB);    // difesa
    }

    // FIX goals 0° and 180° (center)  == 90°, 270° funziona ma non sembra per il portiere.
    if((attackGoal >= 85 && attackGoal <= 95)   &&   (defenseGoal >= 265 && defenseGoal <= 275)) {
      *coordX = 0; 
      *coordY = UNDEFINED_COORD; 
    }
    else {  

      // converto i gradi in radianti
      attackGoal  = ToRad(attackGoal);
      defenseGoal = ToRad(defenseGoal);

      // conefficienti angolari
      float m1 = tan(attackGoal);
      float m2 = tan(defenseGoal);

      // calcolo la posizione nel campo utilizzado gli angoli
      float xcal = -100 / (m1 - m2);
      float ycal = m1 * xcal + 50;

      *coordX = (int) xcal;
      *coordY = (int) ycal;
    }
  }
  else {   // se non vedo porte  
    *coordX = COORD_NOT_FOUND; 
    *coordY = COORD_NOT_FOUND;
  }
  
}



// ritorna la direzione per andare in un punto di coordinate (coordX, coordY)
int GoToCoords(int coordX, int coordY, int robotX, int robotY) {
  // coordX = coordinata X del punto di destinazione
  // coordY = coordinata Y del punto di destinazione
  // robotX = coordinata X del robot nel campo
  // robotY = coordinata Y del robot nel campo

  // calcolo angolo della direzione (arcotangente di deltaX e deltaY) in radianti.
  double angDirection = atan2((double)(coordY - robotY), (double)(coordX - robotX));
  
  // converto in gradi
  int dir = ToDegree(angDirection);

  // converto in gradi del robot
  dir = ToRobotAngles(dir);

  // FIX: ROBOT AL CENTRO 
  if((robotX == 0) && (robotY == UNDEFINED_COORD))  {
    if (coordX > 0)  dir = 90;
    else dir = 270; 
  }

  // ritorno angDirection tipo intero
  return dir;
}


// compute the distance from the robot to a point (x, y)
unsigned int computeDistance(int coordX, int coordY, int robotX, int robotY) {

  // compute distance 
  int deltaX = coordX - robotX;
  int deltaY = coordY - robotY;

  if(deltaX < 0) deltaX *= -1;
  if(deltaY < 0) deltaY *= -1;

  unsigned int distance = (unsigned int) sqrt(deltaX*deltaX  +  deltaY*deltaY); 

  // fix Y undefined
  if(robotX == 0 && robotY == UNDEFINED_COORD && coordX == 0) {
    distance = 0; 
  }

  return distance;
}



// go to the center of the field 
// funziona ma vai piano, meglio linesToCenter() se devo andare al centro 
void gotoCenter(struct data dataRobot, int *dir, int *vel, int *orient) {

  static unsigned long timerCenter = 0; 
  
  unsigned int dist = computeDistance(0,0,dataRobot.coordX, dataRobot.coordY);

  *orient = 0; 
  *dir = linesToCenter(dataRobot);
  *vel = VEL_GOTOCENTER_MAX;

 
  if(dist < 15) {
    *vel = VEL_GOTOCENTER_MIN; 
    timerCenter = millis(); 
  }
  else {
    if((millis() - timerCenter) < 500)  *vel = VEL_GOTOCENTER_MIN; 
  }

}



// go to a point in the field 
bool goToPoint(struct data dataRobot, int pointX, int pointY, int *dir, int *vel) {
  
  unsigned int d = computeDistance(pointX, pointY, dataRobot.coordX, dataRobot.coordY);
  static unsigned long time = millis(); 

  // direction 
  *dir = GoToCoords(pointX, pointY, dataRobot.coordX, dataRobot.coordY);
  *dir = fixOrient(*dir, dataRobot.compass);  

  // undefined Y 
  if(pointX == 0 && dataRobot.coordX == 0 && dataRobot.coordY == UNDEFINED_COORD) d = 0; 
  
  // velocity 
  *vel = 30 + (d * 4);
  if(*vel > VEL_GOTOPOINT_MAX)  *vel = VEL_GOTOPOINT_MAX;

  // stop 
  if(d < 4) *vel = 0; 
  else time = millis(); 
  
  // arrived 
  if((millis() - time) > 500) return true; 
  else return false; 
}


/*
int filterAngCAM_blue(int ang) {
  
  static int ang_prec = 0; 
  int angFilter = 0; 

  if(((ang > 180) && (ang_prec < 180))  ||  (((ang < 180) && (ang_prec > 180)))) {
    if(ang > 180) ang = -(360 - ang);
    else if(ang_prec > 180) ang_prec = -(360 - ang_prec);

    angFilter = (ang * COEFF_GOAL_POSITION) + (ang_prec * (1 - COEFF_GOAL_POSITION));
  }
  else {
    angFilter = (ang * COEFF_GOAL_POSITION) + (ang_prec * (1 - COEFF_GOAL_POSITION));
  }

  if(angFilter < 0) angFilter += 360;
  ang_prec = angFilter;

  return angFilter;
}
*/


// complementary filter of goal angles for position system 
int filterAngCAM_yellow(int ang) {
  
  static int ang_prec = 0; 
  int angFilter = 0; 

  if(((ang > 180) && (ang_prec < 180))  ||  (((ang < 180) && (ang_prec > 180)))) {
    if(ang > 180) ang = -(360 - ang);
    else if(ang_prec > 180) ang_prec = -(360 - ang_prec);

    angFilter = (ang * COEFF_GOAL_POSITION) + (ang_prec * (1 - COEFF_GOAL_POSITION));
  }
  else {
    angFilter = (ang * COEFF_GOAL_POSITION) + (ang_prec * (1 - COEFF_GOAL_POSITION));
  }

  if(angFilter < 0) angFilter += 360;
  ang_prec = angFilter;

  return angFilter;
}