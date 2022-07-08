#include "vars.h"

// utility functions to pritn data without overload the serial monitor
void Print(String data_string, unsigned int timer) {
  static unsigned long t = millis();
  if((millis() - t) > timer)  {
      Serial.print(data_string);
      t = millis(); 
  }
}   

// print with new line character included
void Println(String data_string, unsigned int timer) {
  static unsigned long t = millis();
  if((millis() - t) > timer)  {
      Serial.println(data_string);
      t = millis(); 
  }
}






/*Variabile diagramma a stato

int STATE = ATTACK;
unsigned long t;

//seni e coseni
float sins[360], cosin[360];

//palla
int ballDist, ballDeg;

//GlobalMov
int globalDir, globalVel;

// bytes attivazione sensori di linea attuale e precedente
byte Sread;
byte Sprec; 
*/