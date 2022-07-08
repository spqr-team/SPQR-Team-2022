// TEST BALL con max and min values sensors e considero next e prev flavio

/**
  sensorNumber Mapping
    sensorNumber  Angle Pin Port
    S1      0     26  PD6
    S2            25  PD4
    S3            19  PD1
    S4            18  PD0
    S5      90    1   PE6
    S6            40  PF1
    S7            39  PF4
    S8            38  PF5
    S9      180   37  PF6
    S10           36  PF7
    S11           32  PC7
    S12           31  PC6
    S13     270   30  PB6
    S14           29  PB5
    S15           28  PB4
    S16           27  PD7
    loop cycle duration: 3.2 millis
**/


// reading sensorNumbers faster by PORT register
#define S5 ((PINE & 64 ) >> 6)

#define S11 ((PINC & 128 ) >> 7)
#define S12 ((PINC & 64 ) >> 6)

#define S13 ((PINB & 64 ) >> 6)
#define S14 ((PINB & 32 ) >> 5)
#define S15 ((PINB & 16) >> 4)

#define S1 ((PIND & 64 ) >> 6)
#define S2 ((PIND & 16 ) >> 4)
#define S3 ((PIND & 2 ) >> 1)
#define S4 (PIND & 1 )
#define S16 ((PIND & 128) >> 7)

#define S6 ((PINF & 2 ) >> 1)
#define S7 ((PINF & 16 ) >> 4)
#define S8 ((PINF & 32 ) >> 5)
#define S9 ((PINF & 64 ) >> 6)
#define S10 ((PINF & 128 ) >> 7)


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

// TEST massimi e minimi 
int max[16] = {0};
int min[16] = {0};

unsigned long sendtimer = 0;

float angle; 
float xs[16];
float ys[16];


#define COEFF 0.3


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
  
  /*pinMode(26, INPUT);   //S1
  pinMode(25, INPUT);   //S2
  pinMode(19, INPUT);   //S3
  pinMode(18, INPUT);   //S4
  pinMode(1, INPUT);    //S5
  pinMode(40, INPUT);   //S6
  pinMode(39, INPUT);   //S7
  pinMode(38, INPUT);   //S8
  pinMode(37, INPUT);   //S9
  pinMode(36, INPUT);   //S10
  pinMode(32, INPUT);   //S11
  pinMode(31, INPUT);   //S12
  pinMode(30, INPUT);   //S13
  pinMode(29, INPUT);   //S14
  pinMode(28, INPUT);   //S15
  pinMode(27, INPUT);   //S16*/
  
  for (int i = 0; i < 16; i++) {
    xs[i] = cos((22.5 * PI / 180) * i);
    ys[i] = sin((22.5 * PI / 180) * i);
  }
}

// valore precedente 
int angle_prec = 0;


void loop() { 
  
  // reset counters
  for (byte i = 0; i < 16; i++) {
    counter[i] = 0;
  }

  //reads ball sensors from registers 
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

    
  // printo i valori dei sensori 
  for (byte i = 0; i < 16; i++) {
      Serial1.print(String(i));
      Serial1.println(": " + String(counter[i]));
  }
  

  // max and min values
  for (byte i = 0; i < 16; i++)  {
    if(counter[i] > max[i]) max[i] = counter[i];
    if(counter[i] < min[i]) min[i] = counter[i];
  }

  // printo max 
  /*
  Serial1.print("\nmax: ");
  for (byte i = 0; i < 16; i++)  {
    Serial1.print(String(i) + ":" + String(max[i]) + " ");
  }
  */
  
  // printo min
  /* 
  Serial1.print("\nmin: ");
  for (byte i = 0; i < 16; i++)  {
    Serial1.print(String(i) + ":" + String(min[i]) + " ");
  }
  */

    
  static float x = 0.0;
  static float y = 0.0;
  x = 0.0; 
  y = 0.0;

  // filter values too high or too low
  maxCounter = 0;
  int nextS;
  int prevS;
  
  for (byte i = 0; i < 16; i++) {
    if (counter[i] > TOO_HIGH || counter[i] < TOO_LOW) counter[i] = 0;

    x += xs[i] * counter[i];
    y += ys[i] * counter[i];

    if (counter[i] > maxCounter) {
      maxCounter = counter[i];
      sensorNumber = i;
    }

  }


  /*
  nextS = (sensorNumber + 1) % 16;
  prevS = ((sensorNumber - 1) + 16) % 16; 
 
  x += xs[sensorNumber] * counter[sensorNumber];
  y += ys[sensorNumber] * counter[sensorNumber];
  x += xs[nextS] * counter[nextS];
  y += ys[nextS] * counter[nextS];
  x += xs[prevS] * counter[prevS];
  y += ys[prevS] * counter[prevS];
  */
  
  // ball distance
  distance = maxCounter;

  
  // compute angle of the ball
  angle = atan2(y, x) * 180 / PI;
  angle = ((int)(angle + 360)) % 360;

  
  // angle filter 
  int angleFilter = (angle * COEFF) + (angle_prec * (1 - COEFF));
  angle_prec = angleFilter;

  
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

  // send angle over serial 
  //Serial1.println("\nang:  " + String(str_angle));
   
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
