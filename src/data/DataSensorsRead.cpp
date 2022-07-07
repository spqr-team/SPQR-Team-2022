#include <Arduino.h>
#include "sensors/imu.h"
#include "data/DataSensorsRead.h"
#include "sensors/ballRead.h"
#include "sensors/lines.h"
#include "sensors/camera.h"
#include "bluetooth.h"
#include "vars.h"

// legge tutti i sensori e inserisce i dati nella struttura dati grezzi
void readDataSensors(struct inputData * SensorData) {
    
    // IMU
    float currentIMU = readIMU();
    SensorData->compass = currentIMU;


    // ball
    static int angle = 0;
    static int distance = 0;
    readBall(&angle, &distance);
    SensorData->angleBall = angle;
    SensorData->distanceBall = distance;


    // camera
    int angBlue = BLOB_NOT_FOUND;   // angle blue
    int angYellow = BLOB_NOT_FOUND; // angle yellow
    int areaYgoal = 0;              // area yellow 
    int areaBgoal = 0;              // area blu 
    bool newDataCAM = false;        // flag new CAM data
    newDataCAM = readCAM(&angBlue, &angYellow, &areaBgoal, &areaYgoal, (int)currentIMU); // leggo CAM 
    SensorData->angBlu = angBlue;
    SensorData->angYellow = angYellow;
    SensorData->areaB = areaBgoal;
    SensorData->areaY = areaYgoal; 
    SensorData->newDataCAM = newDataCAM;


    // test seriale CAM
    /*
    if(newDataCAM) {
        Serial.println("angB: " + String(angBlue));
        Serial.println("angY: " + String(angYellow));
        Serial.println("areaB: " + String(areaBgoal));
        Serial.println("areaY: " + String(areaYgoal));
    }
    */

   // test palla
   //Serial.println("ang: " + String(SensorData->angleBall));
   //Serial.println("dist: " + String(SensorData->distanceBall));
}
