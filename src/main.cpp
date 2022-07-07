#include <Arduino.h>
// PORTIERE RICK FINITO
// ATTENZIONE da risolvere bug sulla porta blu
// fix bug entra nell'area se vedo la linea con SI o SE dentro l'area. 
// provo velocità minima maggiore: meglio 
// da aggiungere presa palla al portiere 
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


// ---- PORTIERE FINITO (da testare in gioco vero) ----
// fixato oscillazione in Y e esco se entrato completamente portiere in vectY_lines() 
// aggiunto no spazzata su ritorno in porta: OK
// aggiunto spazzata laterale più frequente: OK 
// fix non entra nell'area mentra spazza ai lati: meglio ma ancora da migliorare 
// da fare gestione caso palla dietro: da fare sul ritorno in porta: 
// aggiunto no spazzata se palla dietro e portiere ai lati dell'area: OK 
// - testare fix bug entrato completamente: OK
// - testare torno indietro piano dopo spazzata: OK
// - ritorno in porta: ok, forse megliocon posizione 
// - testare spazzata ai lati della porta, meno pausa spazzata (600ms): meglio  
// - aggiunto coeff vectY su vectY_lines()
// portiere ok, da fare spazzata ai lati dell'area
// ---------------------------------------------------

// ball_read_remap e tracking_porte_rick
// bno e roller ok 
// aggiunto vai piano dopo spazzata, filtro orient e configrata meglio distanza palla portiere
// da fare fix attaccnate che si blocca sulla linea dell'area 
// aggiunto ritorno al centro porta se non vedo palla (compute() toCenter) 
// da fare palla dietro al portiere 
// test ritorno centro porta se non vedo palla: ok funziona entra parecchio  
// test spazzata e configurare dist palla: ok funziona
// funzione goToPoint(): ok funziona, l'ho modoficiata
// test delle strategie con roller:   

// portiere cose da migliorare:
// - ritorno al centro della porta se non vedo palla  
// - spazzata piccola con attaccante e forse lunga senza.
// - fix linee 

// ATTACCANTE BASE FATTO, da aggiungere roller e kicker
// ritorno al centro se non vedo palla FUNZIONA
// rampa linee FUNZIONA: meglio
// presa palla non funziona ritorno a quella vecchia  

// presa palla funionicchia ma spesso va addosso alla palla coeff lento e no isteresi: va abbastanza bene  
// cono anteriore ok, ristringo da 20 a 12 ok 
// provo a aumentare zona VERY_CLOSE: mda 105 a 117 OK
// provo flag timer_coeff: meglio
// presa palla funziona bene tranne dietro da fixare.
// test gotoCenter(): funziona linesTocenter, vai piano!
// da fare rampa linee e fix variazione angolo funzione orientToGoal() e media distanza palla 

// testare cono, coeff 360
// testare presa palla, se troppo lenta filtro distanza più veloce
// test coeff goalie con aree distanze: da migliorare  
// aree distanze: 0,1,2 lineari 
// filtro distBall lento  0.01 e offset = 7: meglio da linearizzare le aree distanze
// algoritmo linesToCenter funziona esco solo lato sinistro della porta (specchio), meglio se va più piano 
// testare no ricalcolo: non va bene 
// oggi: scheda, componenti, boundsT, testare velocità linee minore, rampa di velocità  
// da aumentare timer vai piano dopo linee: 
// linee molto più reattive: boundsT alla fine di bounds
// testare fix linesToCenter, caso uscito, rampa: caso robot uscito ok
// testare presa palla con distanza: non funziona, meglio se cambi coeff_goalie con isteresi 
// da committare 

// da migliorare: 
// - linee (testare in velocità, non dobbiamo mai uscire, più reattivi, gestire caso uscito)
// - traiettoria presa palla (con distanza, controllare fix posizione)
// - migliore gestione delle velocità (attaccante più veloce)
// - portiere più reattivo, con spazzata piccola efficace e portiere volante 
// - comunicazione bluetooth e ruoli 
// attaccante vai al centro se non vedi palla è buono 
// poi dopo aver fatto tutto questo si va di strategie di attacco più complesse 

// da scrivere: 
// - led 32u4 
// - controllare area precedente
// - app DEBUG
// - posizione ema distanza porte 
// - protocollo BT per comunicare qualsiasi informazione facile da aggiungere.
// - variazione ang 
// - complementary filter per angoli 
// - filtri su sensori di linea e presa palla  
// - fix portiere linee

// linee: testare in gioco veloce e testare BOUNDS_2
// gestire caso uscito con tutti i sensori: rientro fino a che non ribecco la linea 
// fix angoli posizione più precisi CAM e distanze porte utile ma non fondamentale 
// fix linee area FUNZIONA, da testare in gioco veloce  
// testare sistema linee esco fino a che vedo la linea più reattivo 


// SOFTWARE DI RICK 
// - vettore Y 
// - angoli diminuiti
// - velocità aumentata 

// - ritorno in porta funziona da qualsiasi posizione 
// - non tocco stato linee immacolato funziona per attaccante 
// - esce ancora ai lati da fare cono anteriore
// - provo filtro su velocità portiere: no
// - riduco cono posteriore palla dietro meglio 
// - bug se palla dietro e lontana dall'area non ritorna, provo ritorno se fuori defense: meglio  
// - vel_center minore e più veloce ai lati OK
// - provo fix cono anteriore altrimenti ritorno in porta: funionerebbe ma fatto bene  
// - spazzata piccola: tarata così ok 
// - non esc ma rimane blocccato sulle linee se variazione palla troppo grande
// - provo cono posteriore più piccolo e ritorno in porta MEGLIO
// - fix con  sensori di linea EE e OE  
// - poco più veloce e meno coeffX meglio
// - va meglio valori portiere rick lascio così   
// - spazzata va bene così lascio 
// - problema linee inverto (-70): meglio  
// - aumento limiti DX e SX
// - vai al centro porta se palla lontana: provo con vectY (negativo) 
// - test porta blu: ok  

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
  pinMode(PIN_KICKER, OUTPUT); // pin kicker control 
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

  // ----test----
  tone(BUZZER, 500);
  delay(800);
  noTone(BUZZER);

  delay(2500);
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

