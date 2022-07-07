#include <Arduino.h>
#include "sensors/ballRead.h"
#include "vars.h"


int minVal; // min treshold distance
int maxVal; // max treshold distance
int distances[DISTANCES_NUM] = {VERY_CLOSE, CLOSE, MEDIUM, FAR, NOT_SEE}; // distances areas
int threshold_distances[DISTANCES_NUM] = {117, 135, 165, 180, 250};  // distance areas values (max values)
                                //  prec  105

int distanceBall_threshold(int dist) {
  // determine ball's distance areas
  static int distanceBallArea = distances[0];
  
  // hysteresis
  // ---- salita ----
  maxVal = threshold_distances[distanceBallArea] + OFFSET_THRESHOLD;
  if(dist > maxVal) {
    if(distanceBallArea < (DISTANCES_NUM - 1)) distanceBallArea = distances[distanceBallArea + 1];
  }
  // ---- discesa ----
  if(distanceBallArea == 0) minVal = 0;
  else minVal = threshold_distances[distanceBallArea - 1] - OFFSET_THRESHOLD;
  if(dist < minVal) {
    if(distanceBallArea > 0) distanceBallArea = distances[distanceBallArea - 1];
  }

  // return ball's distance area
  return distances[distanceBallArea];
}



void initBallRead() {
  // seriale arduino mini-pro
  Serial2.begin(57600);
}


void readBall(int *angle, int * distance) {
  // read ball's diastance and angle from ATmega32u4 on board.

  bool startB = false;    // start byte
  bool endB = true;       // end byte
  byte data = 0;          // byte received
  long data_received = 0; // data byte received
   
  // finchè ci sono dati del buffer seriale  
  while (Serial2.available()) {
    // leggo un byte 
    data = Serial2.read();
    
    // start pacchetto
    if (((data == 'A') || (data == 'D'))  &&  (endB == true)  &&  (startB == false)) {      
      startB = true;
      endB = false; 
      data_received = Serial2.parseInt();
    }
      
    // end pacchetto
    else if (((data == 'a') || (data == 'd'))  &&  (startB == true)  &&  (endB == false)) {   
      startB = false;
      endB = true; 
      // ANGLE
      if(data == 'a')   {
        *angle =  (int)(data_received);
      }
      // DISTANCE
      if(data == 'd') {
        *distance = (int)(data_received);
      }
    }
  }
}



int filterBallAngle(int ang) {
  // filtro complementare angolo palla lineare
  static int ang_prec = 0; 
  int angFilter = 0; 

  if(((ang > 180) && (ang_prec < 180))  ||  (((ang < 180) && (ang_prec > 180)))) {
    if(ang > 180) ang = -(360 - ang);
    else if(ang_prec > 180) ang_prec = -(360 - ang_prec);

    angFilter = (ang * COEFF_BALL_ANGLE) + (ang_prec * (1 - COEFF_BALL_ANGLE));
  }
  else {
    angFilter = (ang * COEFF_BALL_ANGLE) + (ang_prec * (1 - COEFF_BALL_ANGLE));
  }

  if(angFilter < 0) angFilter += 360;
  ang_prec = angFilter;

  return angFilter;
}


void readAtmega32u4() {
  // read all data from ATmega32u4
  char c; 

  while (Serial2.available()) {
    c = Serial2.read();
    Serial.print(c);
  }

}
