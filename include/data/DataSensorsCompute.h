// definizione struttura dati elaborati posizione, angPalla, distPalla, ecc...
#include <Arduino.h>
#include "sensors/camera.h"
#include "systems/position.h"
#include "vars.h"


struct data {
  // IMU
  float compass;


  // BALL
  int angleBallC;         
  int AngleBallC_absolute;
  int distanceBallC;
  int ballDist_thresholds;


  // LINES: elaboro direttamente i sensori di linea senza lettura in (inputData)
  bool FLAG_lines = false;         // flag lines detected
  byte lineSensors_current = 0;    // active current line sensors 
  byte lineSensors_last = 0;       // byte with the previous value of lineSensors_current
  byte lineSensors_activated_number = 0;  // number of linesensors activated now and in the past 
  byte lineSensors_activated_byte = 0;    // byte of line sensors activated now and in the past
  byte lineSensors_prec = 0;       // line sensors activated previously
  byte lineSensors_total = 0;      // total line sensors activated
  bool resetActivatedSens = false; // reset activated sensors
  int  exitAngLines;               // exit angle to escape from bounds


  // CAM
  int  angBlueC = BLOB_NOT_FOUND;
  int  angYellowC = BLOB_NOT_FOUND;
  int  angBlue_prec = BLOB_NOT_FOUND;
  int  angYellow_prec = BLOB_NOT_FOUND;
  int  areaBlueC = 0; 
  int  areaYellowC = 0; 
  int  areaBlue_prec = 0; 
  int  areaYellow_prec = 0; 
  unsigned long timerBlue = millis();
  unsigned long timerYellow = millis();
  bool newBLOB_blue;
  bool newBLOB_yellow;
  bool old_angBlue;
  bool old_angYellow;


  // ENEMY'S GOAL 
  bool attackGoal;
  // defense
  int  defenseArea = 0; 
  int  defenseAng = BLOB_NOT_FOUND; 
  // attack
  int  attackArea = 0; 
  int  attackAng = BLOB_NOT_FOUND;



  // POSITION
  bool prevPosition = false;           // flag considering previous position 
  bool positionAvailable = false;      // flag position data available
  int coordX = COORD_NOT_FOUND;        // coordinata X del robot
  int coordY = COORD_NOT_FOUND;        // coordinata Y del robot
  int coordX_prec = COORD_NOT_FOUND;   // previous coordinate 
  int coordY_prec = COORD_NOT_FOUND;   // previous coordinate
  unsigned long timePosition_data = 0; // reliability timer position data

  // ROLE and MODE
  byte role;
  byte mode; 
  bool NOS = false;
  
}; 



void computeDataSensors(struct inputData inputDataRobot,  struct data * dataRobot);
