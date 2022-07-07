// test ball scheda roller 
// ball read 32u4 di roller 
// i sensori di palla della scheda di roller sono shiftati di due posizioni 
// rispetto a rick. I sensori sono stati cambiati via software.
// si parte dal sensore S0 è quello davanti la bocca del robot e si gira in senso orario.
// loop cycle duration: 3.2 millis

// sensors mapping to 32u4 pins (scheda di roller)
//  n. sens | pin s32u4
#define S0 28
#define S1 27
#define S2 26
#define S3 25
#define S4 19
#define S5 18
#define S6 1
#define S7 40
#define S8 39
#define S9 38
#define S10 37
#define S11 36
#define S12 32
#define S13 31
#define S14 30
#define S15 29


// reading sensorNumbers faster by PORT register
/*
#define S3 ((PINE & 64 ) >> 6)

#define S9 ((PINC & 128 ) >> 7)
#define S10 ((PINC & 64 ) >> 6)

#define S11 ((PINB & 64 ) >> 6)
#define S12 ((PINB & 32 ) >> 5)
#define S15 ((PINB & 16) >> 4)

#define S13 ((PIND & 64 ) >> 6)
#define S14 ((PIND & 16 ) >> 4)
#define S1 ((PIND & 2 ) >> 1)
#define S2 (PIND & 1 )
#define S16 ((PIND & 128) >> 7)

#define S4 ((PINF & 2 ) >> 1)
#define S5 ((PINF & 16 ) >> 4)
#define S6 ((PINF & 32 ) >> 5)
#define S7 ((PINF & 64 ) >> 6)
#define S8 ((PINF & 128 ) >> 7)
*/

#define NCYCLES  250    // number of readings (MAX = 255)
#define TOO_HIGH 230    // TOO_HIGH values
#define TOO_LOW   45    // TOO_LOW  values

// init LEDS
#define LED1ON (PORTD = PORTD | 0b00100000)
#define LED1OFF (PORTD = PORTD & 0b11011111)

#define LED2ON (PORTF = PORTF | 0b00000001)
#define LED2OFF (PORTF = PORTF & 0b11111110)

#define LED3ON (PORTB = PORTB | 0b10000000)
#define LED3OFF (PORTB = PORTB & 0b01111111)

#define LED4ON (PORTB = PORTB | 0b00000001)
#define LED4OFF (PORTB = PORTB & 0b11111110)


byte counter[16];
byte distance;
byte maxCounter = 0;
byte sensorNumber = 0;

unsigned long sendtimer = 0;

float angle; 
float xs[16];
float ys[16];


void setup() {
  delay(1000);

  // init serial ATmega32u4-teensy
  Serial1.begin(57600);

  //For now replace pinMode with writes to the direction register. 
  //We don't know if pinMode will work on those sensorNumbers, and it has proven not be working on digitalWrite for reasons probably relative to compatibility between Arduino and our board,
  //but this needs further investigation*/  
  //Set the LEDs as outputs, keep the rest as input by default

  //LED3(PB7) and LED4 (PB0)
  DDRB |= 0b10000001;
  //LED2 (PF0)
  DDRF |= 0b00000001;
  //LED1 (PD5)
  DDRD |= 0b00100000;
  
  pinMode(S0, INPUT);   //S1
  pinMode(S1, INPUT);   //S2
  pinMode(S2, INPUT);   //S3
  pinMode(S3, INPUT);   //S4
  pinMode(S4, INPUT);    //S5
  pinMode(S5, INPUT);   //S6
  pinMode(S6, INPUT);   //S7
  pinMode(S7, INPUT);   //S8
  pinMode(S8, INPUT);   //S9
  pinMode(S9, INPUT);   //S10
  pinMode(S10, INPUT);   //S11
  pinMode(S11, INPUT);   //S12
  pinMode(S12, INPUT);   //S13
  pinMode(S13, INPUT);   //S14
  pinMode(S14, INPUT);   //S15
  pinMode(S15, INPUT);   //S16
  
  for (int i = 0; i < 16; i++) {
    xs[i] = cos((22.5 * PI / 180) * i);
    ys[i] = sin((22.5 * PI / 180) * i);
  }
}



void loop() { 
  
  // reset counters
  for (byte i = 0; i < 16; i++) {
    counter[i] = 0;
  }

  //reads ball sensors from registers 
  for (byte i = 0; i < NCYCLES; i++) {
    counter[0] += !digitalRead(S0);
    counter[1] += !digitalRead(S1);
    counter[2] += !digitalRead(S2);
    counter[3] += !digitalRead(S3);
    counter[4] += !digitalRead(S4);
    counter[5] += !digitalRead(S5);
    counter[6] += !digitalRead(S6);
    counter[7] += !digitalRead(S7);
    counter[8] += !digitalRead(S8);
    counter[9] += !digitalRead(S9);
    counter[10] += !digitalRead(S10);
    counter[11] += !digitalRead(S11);
    counter[12] += !digitalRead(S12);
    counter[13] += !digitalRead(S13);
    counter[14] += !digitalRead(S14);
    counter[15] += !digitalRead(S15);
  }

  // printo i valori dei sensori 
  for (byte i = 0; i < 16; i++) {
      Serial1.print(String(i));
      Serial1.println(": " + String(counter[i]));
  }

  

  static float x = 0.0;
  static float y = 0.0;
  x = 0.0; 
  y = 0.0;

  // filter values too high or too low
  maxCounter = 0;
  for (byte i = 0; i < 16; i++) {
    if (counter[i] > TOO_HIGH || counter[i] < TOO_LOW) counter[i] = 0;

    x += xs[i] * counter[i];
    y += ys[i] * counter[i];

    if (counter[i] > maxCounter) {
      maxCounter = counter[i];
      sensorNumber = i;
    }
  }

  // ball distance
  distance = maxCounter;

  // compute angle of the ball
  angle = atan2(y, x) * 180 / PI;
  angle = ((int)(angle + 360)) % 360;

  //distance is 0 when not seeing ball
  //dist = hypot(x, y);
  //turn led on if the robot is seeing ball
  if (distance == 0) {
    LED1OFF;
  } else {
    LED1ON;
  }

  // OUTPUT angle and distance to teensy on serial
  // angle: 0-360°
  // distance: 0-NCYCLES (max = 255)
  
  static String str_angle;
  static String str_distance;

  str_angle = String((int) angle, DEC);
  str_distance = String(255 - distance);

  str_angle = 'A' + str_angle + 'a';
  str_distance = 'D' + str_distance + 'd';

  // send angle over serial 
  //Serial1.print(str_angle);
  // send distance over serial 
  //Serial1.print(str_distance);

  //test();
}




// FUNZIONI DI EMA
/**--- READ BALL USING sensorNumberS ANGLE INTERPOLATION ---**/
/*
void readBallInterpolation() {
  
  for (byte i = 0; i < 16; i++) {
    counter[i] = 0;
  }

  //reads from the register
  for (byte i = 0; i < NCYCLES; i++) {
    counter[0] += !S1;
    counter[1] += !S2;
    counter[2] += !S3;
    counter[3] += !S4;
    counter[4] += !S5;
    counter[5] += !S6;
    counter[6] += !S7;
    counter[7] += !S8;
    counter[8] += !S9;
    counter[9] += !S10;
    counter[10] += !S11;
    counter[11] += !S12;
    counter[12] += !S13;
    counter[13] += !S14;
    counter[14] += !S15;
    counter[15] += !S16;
  }

  float x = 0, y = 0;
  maxCounter = 0;
  for (byte i = 0; i < 16; i++) {
    if (counter[i] > TOO_HIGH || counter[i] < TOO_LOW) counter[i] = 0;

    x += xs[i] * counter[i];
    y += ys[i] * counter[i];

    if (counter[i] > maxCounter) {
      maxCounter = counter[i];
      sensorNumber = i;
    }
  }

  distance = maxCounter;

  angle = atan2(y, x) * 180 / PI;
  angle = ((int)(angle + 360)) % 360;

  //distance is 0 when not seeing ball
  //dist = hypot(x, y);

  //turn led on if the robot is seeing ball
  if (distance == 0) {
    LED1OFF;
  } else {
    LED1ON;
  }
}



unsigned long sendtimer = 0;

void sendDataInterpolation() {
  //We must ensure that each number is expressed with three digits, adding leading zeros if necessary, to further improve transmission reliability
  String str_angle = String((int) angle, DEC);
  String str_distance = String(NCYCLES - (int) distance, DEC);

  String sendStr_angle = "";
  String sendStr_distance = "";

  if(str_angle.length() == 1) sendStr_angle = "00" + str_angle;
  else if(str_angle.length() == 2) sendStr_angle = "0" + str_angle;
  else sendStr_angle = str_angle;
  
  if(str_distance.length() == 1) sendStr_distance = "00" + str_distance;
  else if(str_distance.length() == 2) sendStr_distance = "0" + str_distance;
  else sendStr_distance = str_distance;

  //slow down the communciation a little bit, or it might get very messy
  if(millis() - sendtimer > 40){
    Serial1.print("b");
    Serial1.print(sendStr_angle);
    Serial1.print("-");
    Serial1.print(sendStr_distance);
    Serial1.print("IB");
    sendtimer = millis();
  }

}


void test() {
  readBallInterpolation();
  
  Serial1.println("===========");
  Serial1.print(S1);
  Serial1.print(" | ");
  Serial1.print(S2);
  Serial1.print(" | ");
  Serial1.print(S3);
  Serial1.print(" | ");
  Serial1.print(S4);
  Serial1.print(" | ");
  Serial1.print(S5);
  Serial1.print(" | ");
  Serial1.print(S6);
  Serial1.print(" | ");
  Serial1.print(S7);
  Serial1.print(" | ");
  Serial1.print(S8);
  Serial1.print(" | ");
  Serial1.print(S9);
  Serial1.print(" | ");
  Serial1.print(S10);
  Serial1.print(" | ");
  Serial1.print(S11);
  Serial1.print(" | ");
  Serial1.print(S12);
  Serial1.print(" | ");
  Serial1.print(S13);
  Serial1.print(" | ");
  Serial1.print(S14);
  Serial1.print(" | ");
  Serial1.print(S15);
  Serial1.print(" | ");
  Serial1.print(S16);
  Serial1.print(" ---  ");
  Serial1.println(sensorNumber);
  Serial1.println("===========");
  delay(100);

}


void printCounter() {
  for (int i = 0; i < 16; i++) {
    Serial1.print(counter[i]);
    Serial1.print(" | ");
  }
    Serial1.print("\t\t| Angle: " );
    Serial1.print(angle);
    Serial1.print("||| Distance: " );
    Serial1.print(distance);
    
  Serial1.println();
  delay(100);
}


void readFromTeensy(){
  while(Serial1.available()){
    byte b = Serial1.read();
    if(b & 0b00000001 == 1) LED2ON;
    else LED2OFF;
    if((b & 0b00000010) >> 1 == 1) LED3ON;
    else LED3OFF;
    if((b & 0b00000100) >> 2 == 1) LED4ON;
    else LED4OFF;
  }
}

*/
