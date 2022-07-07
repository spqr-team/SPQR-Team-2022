// utility functions to easily transform robot's degrees in order to compute all the operations.
// robot's degrees are positive from 0° to 360° considering clockwise

// differenza tra due angoli 
int deltaAng(int angA, int angB);

// from radiants to degrees
int ToDegree(float angRad);

// from degrees to radiants
float ToRad(float angB);

// from goniometric angles to robot's angles
int ToRobotAngles(int ang);

// from robot's angles to goniometric angles
float ToGoniometricAngles(float ang);
