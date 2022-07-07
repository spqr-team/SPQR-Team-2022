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
#define SOGLIA 260  // soglia lab = 260  // soglia casa = 480 // romecup = 500

// correzione linee timer sensori attuali
#define TIMER_LINE_SENSORS 400


byte readLines_current();

byte readLines_total(byte Sprec, byte Scurrent);

byte LineSensorsActivated_byte(byte Scurrent,bool reset);

byte LineSensorsActivated_number(byte Scurrent,bool reset);

int  readAngLines(byte Stotal, float orient);

int linesToCenter(struct data dataRobot);

int linesVelRamp(unsigned long ttime);

void checkLineSensors();

void lineSensorsTest();

void readAllSensors();