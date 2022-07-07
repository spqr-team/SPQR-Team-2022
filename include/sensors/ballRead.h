// timeout serial communication with 32u4 (ms)
#define TIMEOUT_BALL_DATA 100

// flag if timeout expires
#define NOT_BALL_DATA_AVAILABLE 666

// coefficient complementary filter of ball's distance
#define FILTER_BALL_DISTANCE 0.01 // prec = 0.1

// coefficient linear complementary filter of ball's angle precedente = 0.2
#define COEFF_BALL_ANGLE 0.9 // prec = 0.2

// number of distance areas of the ball
#define DISTANCES_NUM 5

// different distance ares of the ball
#define VERY_CLOSE  0
#define CLOSE       1
#define MEDIUM      2
#define FAR         3
#define NOT_SEE     4 

// offset min and max threshold of ball's distance areas 
#define OFFSET_THRESHOLD 10


// compute ball's distance area
int distanceBall_threshold(int dist);

// init serial atmega32u4
void initBallRead();

// reading angle and distance of the ball
void readBall(int * ang, int * dist);

// complementary linear filter funztion 
int filterBallAngle(int ang);

// read all data (test)
void readAtmega32u4();
