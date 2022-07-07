#include "PID_v2.h"

double Kp=1, Ki=0, Kd=0;
double Input, Output,Setpoint;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // put your setup code here, to run once:
  myPID.SetSampleTime(1.5);
  myPID.SetDerivativeLag(1);
  myPID.SetOutputLimits(-255,255);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetMode(AUTOMATIC);
  Serial.begin(9600);
}

void loop() {
  Input = 0; //[-179,180]
  Setpoint = 0;
  myPID.Compute();
  Serial.println(Output);
  
}
