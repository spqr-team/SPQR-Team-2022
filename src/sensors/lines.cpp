#include <Arduino.h>
#include "sensors\lines.h"
#include "systems\position.h"
#include "data/DataSensorsCompute.h"
#include "data/DataSensorsRead.h"
#include "systems/degreesRemapping.h"
#include "vars.h"

// pinout line sensors: SIE/OIE/NIE/EIE 
int linesensAll[8] = {15, 14, 23, 22, 20, 21, 17, 16}; 

// array dei contributi assi X e Y per direzione dei sensori di linea (NORD, SUD, ecc...)
// bit corrispondenti sensori {15,13,11,9,7,5,3,1}
// ogni sensore ha un corrispondente contributo X e Y, sono sommati e si calcola l'angolo con la funzione 
// arcotangente atan2() in radianti da convertire in gradi.
int SlineaX [8] =  {0, 0, 20, 40, 0, 0, -20, -40};
int SlineaY [8] =  {20, 40, 0, 0, -20, -40, 0, 0};

// variabile globale contatore sensori che si sono attivati nella storia 
int sensoriAttivi = 0; 

//LINEE LATINO
int linepinsI[4] = {20, 17, 15, 23};
int linepinsO[4] = {21, 16, 14, 22};
int linetriggerI[4];
int linetriggerO[4];
byte linesensbyteI;
byte linesensbyteO;
byte linesensbyte;

/*
void checkLineSensors() {
  linesensbyteI = 0;
  linesensbyteO = 0;

  for(int i = 0; i < 4; i++) {
    linetriggerI[i] = analogRead(linepinsI[i]) > SOGLIA;
    linetriggerO[i] = analogRead(linepinsO[i]) > SOGLIA;
    linesensbyteI = linesensbyteI | (linetriggerI[i]<<i);
    linesensbyteO = linesensbyteO | (linetriggerO[i]<<i);
  }
  if ((linesensbyteO > 0)){
    timerLinee = 0;
  }

  linesensbyte |= (linesensbyteI | linesensbyteO);
  Serial.println(linesensbyte);

  //DOVE 1 N / 2 E / 4 S / 8 O
  if(timerLinee <= 255) {
    tone(BUZZER, 255);
    if(linesensbyteO == 1) {
      globalDir = 180;
      globalVel = 355;
    }
    else if(linesensbyteO == 2) {
      globalDir = 270;
      globalVel = 355;
    }
    else if(linesensbyteO == 4) {
      globalDir = 0;
      globalVel = 355;
    }
    else if(linesensbyteO == 8) {
      globalDir = 90;
      globalVel = 355;
    }
  } else noTone(BUZZER); 
}
*/

byte LineSensorsActivated_byte(byte Scurrent,bool reset) {
  // returns the byte of activated sensors now and in the past
  
  static byte activatedSensors = 0; 

  // reset active sensors
  if(reset == true) activatedSensors = 0;

  // OR to track all activated sensors
  activatedSensors = Scurrent | activatedSensors;

  return activatedSensors;
}


byte LineSensorsActivated_number(byte Scurrent,bool reset) {
  // count the number of activated sensors and reset 

  static byte activeSensors = 0; 
  byte count = 0;

  // reset active sensors
  if(reset == true) activeSensors = 0;

  // OR to track all activated sensors
  activeSensors = Scurrent | activeSensors;
  
  // count activated sensors
  for (byte i = 0; i < 8; i++) {
    if(activeSensors & (1 << i)) count++;
  }

  return count;
} 


byte readLines_current() {
  // read current active line sensors

  byte Sread = 0; 

  // read all line sensors {15, 14, 23, 22, 20, 21, 17, 16}  SIE/OIE/NIE/EIE
  for (byte i = 0; i < 8; i++) {
    if (analogRead(linesensAll[i]) > SOGLIA) Sread |= (1 << i);
  }

  // return the byte sensors
  return Sread;
}


byte readLines_total(byte Sprec, byte Scurrent) {
  // return a byte with all sensors activated in the past and in the present (OR), the direction will
  // be computed on this byte

  byte total_sensors = 0;

  // flag max number of line sensors achieved.
  static bool flag_maxSensors = false;
  
  // reset flag max line sensors
  if(Sprec == 0) flag_maxSensors = false;

  // conto sensori attivati nel byte Sprec (storia dei sensori)
  byte sens_prec = 0; 
  for(int c = 0; c < 8; c++) {
    if(Sprec & (1 << c)) sens_prec++;
  }

  // conto sensori attivi attuali
  byte sens_current = 0; 
  for(int c = 0; c < 8; c++) {
    if(Scurrent & (1 << c)) sens_current++;
  }

  // FIX OR linee
  // OR tra Scurrent e Sprec se la somma è minore o uguale a 6, altrimenti ritorno total_sensors = Sprec
  if(((sens_current + sens_prec) <= 6)  &&  (flag_maxSensors == false))   total_sensors = Scurrent | Sprec;
  else {
    total_sensors = Sprec;
    flag_maxSensors = true;
  }
  
  // ritorno il byte con i sensori di linea e i loro rispettivi valori
  return total_sensors;
}


int readAngLines(byte Stotal, float orient) {
  // compute the angle to escape from lines 

  // variabili somme dei contributi
  int vectX = 0;
  int vectY = 0;

  // angolo di uscita
  float ExitAng;

  // per ogni bit (sensore) del byte Sread, se il bit vale 1 (sensore attivo) sommo i suoi contributi
  // X o Y alle variabili vectX o vectY (vettori x e y)  
  if (SI)  vectY += 40;  // bit 0
  if (SE)  vectY += 20;  // bit 1
  if (OI)  vectX += 40;  // bit 2
  if (OE)  vectX += 20;  // bit 3
  if (NI)  vectY += -40; // bit 4
  if (NE)  vectY += -20; // bit 5
  if (EI)  vectX += -40; // bit 6
  if (EE)  vectX += -20; // bit 7

  // calcolo angolo di uscita con funzione arcotangente dei vettori vectX e vectY
  // e converto il risultato da radianti in gradi. poi scalo il valore dell' angolo rispetto 
  // allo 0° del robot, quindi considero 90° come lo zero e sottraggo ExitAng perchè 
  // il verso di rotazione considerato dal robot è contrario.
  // se eAng è negativo allora sommo 360° per riportarlo in positivo, altrimenti calcolo il modulo.
  ExitAng = atan2(vectY, vectX) *  180 / PI;
  int eAng = 90 - ExitAng;
  if (eAng < 0) eAng += 360;
  else eAng %= 360;
  // scale eAng with the orientation of the robot (orient)
  eAng = (int)(eAng + orient) % 360;
  return eAng;
}


// exit lines go to center
int linesToCenter(struct data dataRobot) {

  int X = dataRobot.coordX; 
  int Y = dataRobot.coordY;
  int centerDir = GoToCoords(0, 0, X, Y);
  int sensorsDir = dataRobot.exitAngLines;
  int direction = 0; 
  
  // se dati posizione incerti rientro con algoritmo sensori e aree porte 
  if(X == UNDEFINED_COORD || Y == UNDEFINED_COORD || X == COORD_NOT_FOUND || Y == COORD_NOT_FOUND) { 

    // rientro verso la porta opposta 
    if((dataRobot.attackArea > dataRobot.defenseArea) && (dataRobot.defenseAng != BLOB_NOT_FOUND))  direction = dataRobot.defenseAng;   
    else if((dataRobot.attackArea < dataRobot.defenseArea) && (dataRobot.attackAng != BLOB_NOT_FOUND)) direction = dataRobot.attackAng;
    else direction = sensorsDir;  

  } 
  else {

    // vai al centro 
    direction = GoToCoords(0, 0, X, Y);

    // se posizione errata in Y rientra verso la porta dietro oppure con i sensori 
    // se differenza tra aree porte è grande
    if(abs(dataRobot.defenseArea - dataRobot.attackArea) > 30)  {
      if(dataRobot.attackArea > dataRobot.defenseArea && Y <= 0)  {
        if(dataRobot.defenseAng != BLOB_NOT_FOUND)  direction = dataRobot.defenseAng; 
        else direction = sensorsDir;
      }
      if(dataRobot.defenseArea > dataRobot.attackArea && Y >= 0) {
        if(dataRobot.attackAng != BLOB_NOT_FOUND) direction = dataRobot.attackAng;
        else direction = sensorsDir; 
      }
    }

  }

  // test 
  /*
  Serial.println("X: " + String(X));
  Serial.println("Y: " + String(Y));
  Serial.println("defenseArea: " + String(dataRobot.defenseArea));
  Serial.println("attackArea: " + String(dataRobot.attackArea));
  Serial.println("defenseAng: " + String(dataRobot.defenseAng));
  Serial.println("attackAng: " + String(dataRobot.attackAng));
  Serial.println("sensorsDir: " + String(sensorsDir));
  Serial.println("centerDir: " + String(centerDir));
  Serial.println("differenza: " + String(abs(dataRobot.attackArea - dataRobot.defenseArea)));
  Serial.println("direction: " + String(direction)); 
  Serial.println("------\n");
  */

  return direction; 
}


// lines velocity ramp
int linesVelRamp(unsigned long ttime)  {
  
  unsigned long dt = abs(millis() - ttime); 
  int vel = 0; 

  if(dt < TIME_RAMP_BOUNDS) vel = (dt * COEFF_VELOCITY_BOUNDS) + BOUNDS_VELOCITY_MIN;
  else vel = BOUNDS_VELOCITY_MAX;

  if(vel > BOUNDS_VELOCITY_MAX) vel = BOUNDS_VELOCITY_MAX;

  return vel; 
}




void lineSensorsTest() {
  byte Stotal = 0; 

  for (byte i = 0; i < 8; i++) {
    if (analogRead(linesensAll[i]) > SOGLIA) Stotal |= (1 << i);
  }

  Serial.println("TEST");
  Serial.println("---------");

  if (SI)  Serial.println("SUD-INTERNO");  // bit 0
  if (SE)  Serial.println("SUD-ESTERNO");  // bit 1
  if (OI)  Serial.println("OVEST-INTERNO");  // bit 2
  if (OE)  Serial.println("OVEST-ESTERNO");  // bit 3
  if (NI)  Serial.println("NORD-INTERNO"); // bit 4
  if (NE)  Serial.println("NORD-ESTERNO"); // bit 5
  if (EI)  Serial.println("EST-INTERNO"); // bit 6
  if (EE)  Serial.println("EST-ESTERNO"); // bit 7

  Serial.println("---------");

}


void readAllSensors() {
  int values[8] = {0}; 

  // media tra i sensori per trovare soglia 
  long somma = 0; 
  for (byte i = 0; i < 8; i++) {
    values[i] = analogRead(linesensAll[i]);
    somma += values[i];
  }

  // media 
  somma = somma / 8; 

  Serial.println("SI: " + String(values[0]));
  Serial.println("SE: " + String(values[1]));
  Serial.println("OI: " + String(values[2]));
  Serial.println("OE: " + String(values[3]));
  Serial.println("NI: " + String(values[4]));
  Serial.println("NE: " + String(values[5]));
  Serial.println("EI: " + String(values[6]));
  Serial.println("EE: " + String(values[7]));
  Serial.println("MEDIA: " + String(somma));

}



/* PER USCITA:
  Anziche' chiamare frenata poi movimento per rientrare:
    - Fai partire un timer come si triggera il primo sensore
    - Dopo aver calcolato uscita come meglio credi, per il primo terzo del tempo di rientro
      mandi a vel massima + 100
      - Secondo terzo a + 50
      - Ultimo a velocita' normale

*/ 


