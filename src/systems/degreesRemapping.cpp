// remapping angles of the robots to compute all the operations and complementary filters
#include <Arduino.h>
#include "systems/degreesRemapping.h" 


// ritorna l adifferenza tra due angoli senza segno 
int deltaAng(int angA, int angB) {

    int delta = 0; 

    if(((angA > 180) && (angB < 180))  ||  (((angA < 180) && (angB > 180)))) {
        if(angA < 180) angA = angA + 360; 
        else angB = angB + 360; 
    }

    delta = abs(angA - angB);
    return delta; 
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


