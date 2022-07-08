#include <Arduino.h>
// a casa: 
// test riconoscimento situazione palla fuori 
// da settare meglio distanza presa palla, e coefficiente filtro
// reset timerlines  
// rotateToAngle non funge

// PRESA PALLA funzionicchia
// ora la traiettoria dell'attaccnate va bene 
// modificata traiettoria su linee area di difesa: meglio con flag SE 
// diagonal kick forse kick in diagonale 
// disattivato centerFlag attaccante(): meglio  
// da fare fix portiere se vedo linea dietro dentro l'area  
// portiere con presa palla 

// presa palla i2c 
// cono anteriore presa palla
// attaccante vel = 90 funziona abbastanza bene ma da risolvere impennata 
// kick_strategy con posizione e angolo porta FUNZIONA.
// vai piano dopo linee: NON FUNZIONA 
// scoperto problema bno 
// velocità max attaccante: 100 
// disabilito orientTogoal con offset angolo porta 
// provo diversi coefficenti attaccante(): NO 
// non considero EE e OE: OK 

// aggiungo taratura automatica soglia linee (init_lines()): FUNZIONA
// controllare vai piano dopo linee
// provare se ho visto linea da poco (200ms) non considero gli esterni e vado verso la palla 
// vado in attack_lines se rimbalzo sulla linea tante volte (contatore)
// risolto bug orient verso palla, devo usare angolo assoluto
// aggiunte funzioni score() e rotate()
// esco piano dalle linee dopo attack_lines: FUNZIONA
// da testare kicK_srtategy: 

// troppi cambi di velocità, da tarare in modo ottimale la traiettoria cono anteriore.
// risolvere presa palla sull'area di difesa 
// presa palla sulla linea 
// forse storcimento plus solo se vicino
// traiettoria presa palla molto meglio 
// attivazione roller molto meglio 
// da gestire presa palla sulle linee
// presaPalla meglio ma ancora ha qualche problema 
// kick se nella metà campo avversaria da testare: 

// testare posizione e direzione linesToCenter() 
// da testare reset attack_lines, con orient e se palla davanti: 
// test attacco sulle linee: problemi buchi tra i sensori 
// provo con tempo di uscita più lungo: molto meglio 
// aggiunto state_prec a BOUNDS_2: NO, non funziona più 
// modificata velocità uscita dalle linee: molto meglio
// aggiungo timer per entrare nello stato ATTACK_LINES: meglio, non esce se è resettato bene   
// attenzione al bug: se reset del conto sensori attivati avviene 
// quando sto sopra la linea, forse può aiutare mettere un timer per ri-entrare
// nello stato ATTACK_LINES

// PROBLEMI ROLLER: 
// - kick inziale da levare sens presa palla() vede 
// - traiettoria palla da modificare 
// fix presa palla sulle linee muovimento solo X o Y 

// PORTIERE RICK FINITO
// da migliorare velocità di rezione, attacco portiere e casi speciali 
// MODIFICATA SOGLIA LINEE e PRESA_PALLA
// PID PORTIERE NON FUNZIONA 
// aggiungo fix se palla dietro portiere: si può fare di meglio  
// sensori presa palla di roller funzionano 
// testare portiere con correzione vectX (PID), funzione init
// testare se non oscilla e trovare i giusto coefficienti 
// aggiungo kick su pin 34 (MULTI2)
// testare controllo kicker e gioco con kicker 
// da fare tiro a giro vicino area avversaria 

// millis loop max: 3.2ms



#define MAIN

#include <Arduino.h>
#include "data/DataSensorsCompute.h"
#include "data/DataSensorsRead.h"
#include "motors/motors.h"
#include "motors/motoHolon.h"
#include "sensors/imu.h"
#include "sensors/ballRead.h"
#include "sensors/lines.h"
#include "strategy/strategy.h"
#include "strategy/striker.h"
#include "strategy/keeper.h"
#include "roller/driveRoller.h"
#include "kicker/driveKicker.h"
#include "sensors/presa_palla.h"
#include "bluetooth.h"
#include "vars.h"


// creo le due strutture dati
struct inputData dataSensors;     // struttura dati grezzi dai sensori
struct data ComputedData;         // struttura dati elaborati



void setup() {
  // ----init----
  pinMode(PIN_PRESA_PALLA, INPUT);    // presa palla 
  pinMode(SW1, INPUT);    // switch libero
  pinMode(SW2, INPUT);    // swicth attack goal
  pinMode(SW3, INPUT);    // switch libero  
  pinMode(PIN_KICKER, OUTPUT);    // pin kicker control 
  digitalWrite(PIN_KICKER, LOW);  // to avoid an initial kick 
  initAttackGoal();       // init attack goal 
  initMotorsPins();       // Init pin, frequenze motori
  initBallRead();         // Init seriale 32u4 lettura palla
  initRoller();           // init esc motor roller
  initHolon();            // init calcoli seni e coseni velocità motori
  initIMU();              // init IMU
  initPID();              // init PID
  Serial.begin(19200);    // serial terminal 
  BLUETOOTH.begin(19200); // bluetooth default baudrate = 19200
  Serial2.begin(57600);   // ATmega32u4 palla
  Serial3.begin(74880);   // openMv h7 CAM
  Serial3.setTimeout(2);  // timeout CAM serial

  delay(2000);
  
  init_lines();   // init soglie sensori di linea 

  // ----test----
  tone(BUZZER, 500);
  delay(800);
  noTone(BUZZER);
}



void loop() { 

  // LEGGO i dati di tutti i sensori e li inserisco in dataSensors (struttura)
  readDataSensors(&dataSensors);

  // ELABORO tutti i dati dei sensori e li inserisco in computedData (struttura) 
  computeDataSensors(dataSensors, &ComputedData);

  // DECIDO che cosa devo fare in base ai dati elaborati (ComputedData)
  // e scrivo direction, velocity in ComputedData
  // orientamento del robot 0° default
  static int orient = 0;
  // direzione di muovimento del robot
  static int direction = 0;
  // velocità del robot
  static int velocity = 0;


  // OUTPUT direzione, velocità, valore letto dalla bussola
  if((computeStrategy(&ComputedData, &direction, &velocity, &orient)) != BRAKE)  {
    go(direction, velocity, orient, ComputedData.compass);
  }


  //BTdebug_send(); 
}

