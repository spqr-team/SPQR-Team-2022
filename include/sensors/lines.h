// bit associated to lines sensors
#define SI (Stotal & 1)   // sud intern
#define SE (Stotal & 2)   // sud extern
#define OI (Stotal & 4)   // ovest intern
#define OE (Stotal & 8)   // ovest extern
#define NI (Stotal & 16)  // nord intern
#define NE (Stotal & 32)  // nord extern
#define EI (Stotal & 64)  // est intern
#define EE (Stotal & 128) // est extern



// lines sensors thresholds
#define SOGLIA 140  // soglia lab roller = 140, rick = 260  // soglia casa rick= 480, roller = 250 

// correzione linee timer sensori attuali
#define TIMER_LINE_SENSORS 400

// numero letture taratura automatica
#define LETTURE 15


byte readLines_current();

byte readLines_total(byte Sprec, byte Scurrent);

byte LineSensorsActivated_byte(byte Scurrent,bool reset);

byte LineSensorsActivated_number(byte Scurrent,bool reset);

int  readAngLines(byte Stotal, float orient);

int linesToCenter(struct data dataRobot);

int linesVelRamp(unsigned long ttime);

void init_lines(); // taratura auto sensori di linea

void checkLineSensors();

void lineSensorsTest();

void readAllSensors();

