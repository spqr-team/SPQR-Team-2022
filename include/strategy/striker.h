#define ANG_TRAIETTORIA 25      // angolo di sfasamento // prec 25, 45
#define DIST_CLOSE     117      // valore soglia distanza
#define COEFF_CLOSE    2.3      // coeff palla vicina     
#define COEFF_FAR      1.8      // coeff palla lontana 
#define COEFF_GOALIE   2.2      // coeff sfasamento 

// tiro a giro e rotate()
#define TIME_DIETRO 500
#define VEL_SPIN    5

// goalie
void attaccante(struct data dataRobot, int * dir);

// storcimento 
void orientToGoal(struct data dataRobot,int * orient);

// init attack goal 
void initAttackGoal();

// tiro a giro parallelo destra o sinistra 
bool score(int compass, bool side, int *orient, int *direction, int *velocity);

// ruota il robot vel variabile 
int rotate(bool sense, float Rvel);

// init orient per funzione rotate 
void initRotate(float bussola);

// ruota il robot fino ad un angolo pino o veloce 
bool rotateToAngle(int *orient, int compass, int setpoint, int delay);



// strategy with kicker 
void kick_strategy(struct data dataRobot);
