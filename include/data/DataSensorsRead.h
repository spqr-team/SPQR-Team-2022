#include <Arduino.h>
// definizione struttura dati sensori linee, palla, CAM e bussola

struct inputData {
  // IMU
  float compass;      // currentIMU value

  // BALL
  int angleBall;      // ball's angle
  int distanceBall;   // ball's distance

  // CAM
  int angBlu;
  int angYellow; 
  int areaB;
  int areaY;
  bool newDataCAM = false;
};


void readDataSensors(struct inputData * SensorData);

