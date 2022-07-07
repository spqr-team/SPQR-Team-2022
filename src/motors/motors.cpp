#include <Arduino.h>
#include "motors/motors.h"
#include "vars.h"


//MOTORI
byte INA_MOT[4] = {12, 25, 27, 29};  // INA pin
byte INB_MOT[4] = {11, 24, 26, 28};  // INB pin
byte PWM_MOT[4] = {4,  5,  2, 3};    // PWM pin


//inizializzo motori
void initMotorsPins() {

  for(int i = 0; i <= 3; i++) {                        
    pinMode(PWM_MOT[i], OUTPUT);
    pinMode(INA_MOT[i], OUTPUT);
    pinMode(INB_MOT[i], OUTPUT);
    analogWriteFrequency(PWM_MOT[i] , PWM_MOTORS);
  }

}

//funzione controllo singoli motori
void mot(byte mot, int vel) { 

  byte VAL_INA, VAL_INB;
  if (vel == 0) {                //se vel 0 motore in folle
    VAL_INA = 0;
    VAL_INB = 0;
  } else if (vel > 0) {          //se positivo motore senso orario
    VAL_INA = 1;
    VAL_INB = 0;
  } else if (vel < 0) {          //se negativo motore senso antiorario
    VAL_INA = 0;
    VAL_INB = 1;
    vel = -vel;
  }
  digitalWriteFast(INA_MOT[mot], VAL_INA);                                                   
  digitalWriteFast(INB_MOT[mot], VAL_INB);
  analogWrite(PWM_MOT[mot], vel);
  return;

}


// test motori
void testMot() {

  mot(0, 100);
  delay(500);
  mot(0, 0);
  delay(500);
  mot(0, -100);
  delay(500);
  mot(0, 0);
  delay(1000);

  mot(1, 100);
  delay(500);
  mot(1, 0);
  delay(500);
  mot(1, -100);
  delay(500);
  mot(1, 0);
  delay(1000);

  mot(2, 100);
  delay(500);
  mot(2, 0);
  delay(500);
  mot(2, -100);
  delay(500);
  mot(2, 0);
  delay(1000);

  mot(3, 100);
  delay(500);
  mot(3, 0);
  delay(500);
  mot(3, -100);
  delay(500);
  mot(3, 0);
  delay(1000);
  
}

// funzione per frenare il robot
void brake() {

  // output motori a = 1, b = 1, PWM = 255
  digitalWriteFast(INA_MOT[0], 1);           
  digitalWriteFast(INB_MOT[0], 1);
  digitalWriteFast(INA_MOT[1], 1);           
  digitalWriteFast(INB_MOT[1], 1);
  digitalWriteFast(INA_MOT[2], 1);           
  digitalWriteFast(INB_MOT[2], 1);
  digitalWriteFast(INA_MOT[3], 1);           
  digitalWriteFast(INB_MOT[3], 1);
  analogWrite(PWM_MOT[0], 255);
  analogWrite(PWM_MOT[1], 255);
  analogWrite(PWM_MOT[2], 255);
  analogWrite(PWM_MOT[3], 255);

}
