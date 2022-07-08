// remapping angles of the robots to compute all the operations and complementary filters
#include <Arduino.h>
#include "systems/degreesRemapping.h" 


// ritorna l'angolo differenza più piccolo tra due angoli 
int deltaAng(int angA, int angB) {

    int delta  = 0; 
    int delta_ = 0; 
    
    if(angA < angB) delta = angA + 360 - angB; 
    else delta = angA - angB; 


    if(angB < angA) delta_ = angB + 360 - angA; 
    else delta_ = angB - angA;

    if(delta <= delta_)  return delta; 
    else return delta_;
}


// from radiants to degrees
int ToDegree(float angRad) {
    return (int)(angRad * 180 / PI);
}

// from degrees to radiants
float ToRad(float angB) {
    return angB * PI / 180.0;
}

// from goniometric angles to robot's angles
int ToRobotAngles(int ang) {
    return (450 - ang) % 360;
}

// from robot's angles to goniometric angles
float ToGoniometricAngles(float ang) {
    if (ang >= 0 && ang <= 90) return (float) 90 - ang;
    else return (float)(90 - ang) + 360;
}


