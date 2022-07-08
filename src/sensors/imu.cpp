#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include "sensors/imu.h"
//#include "vars.h"

elapsedMillis readingIMU;  

Adafruit_BNO055 bno = Adafruit_BNO055();

void initIMU() {
  bno.begin(bno.OPERATION_MODE_IMUPLUS);  //Posizione impostata a P7 alle righe 105,107 di Adafruit_BNO55.cpp
  bno.setExtCrystalUse(true);
}


float readIMU() {
  
  static float imu_current_euler;

  if(readingIMU > 15) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    if (euler.x() != imu_current_euler) {
      imu_current_euler = euler.x();
    }
    readingIMU = 0;
  }
  //return;
  return imu_current_euler;
}




// TEST LETTURA VECCHIA TRE RUOTE 
/*
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);   

int baseIMU = 0;


void initIMU() {
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS))  {
    Serial.print("No BNO055 detected");
    Serial.println("loop bloccato, errore bussola.");
  }
  bno.setExtCrystalUse(true);
  delay(500);
  baseIMU = readIMU();
}

int readIMU() {
  static int ang;

  // DERIVA E CLOCK STRETCHING
  if(readingIMU > 15) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    ang = euler.x() - baseIMU;
    if(ang < 0) ang += 360; 
    readingIMU = 0;
  }
  
  return ang;
}

*/