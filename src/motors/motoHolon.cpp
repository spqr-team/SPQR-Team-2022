#include <Arduino.h>
#include "PID_v2.h" 
#include "vars.h"
#include "motors/motors.h"

float sins[360], cosin[360];

float vx, vy, speed0, speed1, speed2, speed3;

double Anginput, outputMotor, setpoint;

// KP=1.25, KI=0.0, KD=0.1 (costanti mie)
double KP=1.25, KI=0.0, KD=0.1;  // KP=0.8, KI=0, KD=0.025 (costanti EMA)

PID PIDR(&Anginput, &outputMotor, &setpoint, KP, KI, KD, DIRECT);

void go(int direction, int vMot, int orient, float compass) {

    // calcolo moto omnidirezionale 4 motori
    vx = (vMot * cosin[direction]);                    
    vy = (-vMot * sins[direction]);
    speed0 = ((vx * sins[55] ) + (vy * cosin[55] ));   //calcolo velocità dei motori 0,1 (dove 45 e 135 sono i gradi del motore)
    speed1 = ((vx * sins[135]) + (vy * cosin[135]));
    speed2 = ((vx * sins[225]) + (vy * cosin[225]));   //inverto le velocità calcolate per motore 2,3
    speed3 = ((vx * sins[305]) + (vy * cosin[305]));  

    // scale IMU value to 0-180 -180-0 for PID
    if(compass > 180) Anginput = (double)(compass - 360.0);    
    else Anginput = (double) compass;

    // scale desired value of orientation of the robot to 0-180 -180-0 for PID
    if(orient > 180) setpoint = (double) (orient - 360);
    else setpoint = (double) orient;

    // PID correction in order to rotate the robot to orient (angle).
    static double offsetPid = 0; 
    if(PIDR.Compute())  offsetPid = -outputMotor;

    speed0 += offsetPid;
    speed1 += offsetPid;
    speed2 += offsetPid;
    speed3 += offsetPid;

    speed0 = constrain(speed0, -255, 255);
    speed1 = constrain(speed1, -255, 255);
    speed2 = constrain(speed2, -255, 255);
    speed3 = constrain(speed3, -255, 255);

    mot(0,speed0);
    mot(1,speed1);
    mot(2,speed2);
    mot(3,speed3);
}


float torad(float deg) {                      //da gradi a radianti
    return (deg * PI / 180.0);
}


void initHolon() {                            //inizializzo seni e coseni nel setup
    for (int i = 0; i < 360; i++) {   
        sins[i] = sin(torad(i));
    }
    for (int i = 0; i < 360; i++) {
        cosin[i] = cos(torad(i));
    }
}

void initPID() {                     
    // impostazioni PID
    PIDR.SetSampleTime(2);
    PIDR.setAngleWrap(true);        
    // pid->SetDerivativeLag(2);
    PIDR.SetOutputLimits(-255,255);
    PIDR.SetMode(AUTOMATIC);
    PIDR.SetControllerDirection(REVERSE);
}


// fix orient to give absolute direction of the robot.
int fixOrient(int abs_direction, float compass) {
    // decouple orient and direction
    int dir;

    dir = (int)(abs_direction - compass);
    if(dir < 0) dir += 360;

    return dir;
}