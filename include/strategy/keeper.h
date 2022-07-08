// keeper

// max velocity (video test vel = 70)
// 100 e 70 ok al lab 
// 120 e 70 con coeffX = 4 meglio al lab
// rapporto vel sides = center_vel - 30 
#define SIDES_VEL_KEEPER  60  // prec = 70 
#define CENTER_VEL_KEEPER 150 // prec = 100 
#define VEL_BACK_DEFENSE  80  // max = 60 vel = 50 ok

// spazzata piccola 
#define TIME_CATCH 250      // durata spazzata
#define VELOCITY_CATCH 100  // vel spazzata
#define DELAY_SPAZZATA 150  // delay time condizioni spazzata prec = 300
#define PAUSA_SPAZZATA_CENTRO 1500  // pausa tra una spazzata e l'altra
#define PAUSA_SPAZZATA_LATI   600   // pausa spazzata ai lati dell'area 

// min velocity 
#define MIN_VEL_KEEPER  20  // prec = 20 

// complementary filter orient
#define COEFF_ORIENT_KEEPER 0.1 // prec = 0.5

// max ball distance, se la distanza è minore allora fai spazzata 
#define MAX_BALL_DISTANCE   105

// ------YELLOW-------
#define ANG_CENTER_Y  172.0
#define AREA_CENTER_Y 319.0
#define ANG_DX_Y  220.0  // prec = 220.0
#define AREA_DX_Y 232
#define ANG_SX_Y  130.0  // prec = 130.0
#define AREA_SX_Y 232 
// -------------------


// ------BLUE---------
#define ANG_CENTER_B  172 
#define AREA_CENTER_B 301
#define ANG_DX_B  215   
#define ANG_SX_B  140 
// -------------------


/*
#define ANG_CENTER_B  179 
#define AREA_CENTER_B 301
#define ANG_DX_B  242   
#define ANG_SX_B  135

*/

// wrap angle keeper with orient 
#define WRAP_ANGLE 20 // prec = 50, 30, 20  // romecup: 24s

// area di difesa vicino alla porta  
#define AREA_DEFENSE (AREA_CENTER_Y / 3.5) // coeff prec = 3.5

// coeff keeper vector Y (torno indietro verso le linee)
// coefficienti trovati per vel = 100 (3,5)
// vel = 130 (5,7)
#define COEFF_VECT_Y 1.0  // prec = 3, 2.3

// coeff keeper vector X (sensibilità alla palla)
#define COEFF_VECT_X 3.0  // prec = 5, 4

// offset angle infrontof/right/left
#define OFFSET 30

// zoneKeeper
#define CENTER 0
#define SIDES  1





// compute vector X
int Keeper_vectX(struct data dataRobot, bool backToDefense, bool limits);

// compute vector Y
int Keeper_vectY(struct data dataRobot);

// compyte vector Y with area lines
int keeper_vectY_lines(struct data dataRobot); 

// compute keeper direction and velocity
void Keeper_compute(struct data *dataRobot, int *dir, int *vel, int *orient, bool back, bool toCenter);

// turn back to defense goal
bool backTodefense(struct data *dataRobot, int *dir, int *vel);

// complementary filter keeper direction 
int orientFilter(int ang);

// handling lines 
void keeper_lines(struct data *dataRobot, int *vectorY, int *vectorX, byte reactionNum, int linesReact);

// init keeper coefficient and values
void init_keeper(bool attackGoal);

// TEST keeper with orient
void keeper_orient(struct data dataRobot, int *dir, int *vel, int *orient);

// TEST lines
bool lines(struct data dataRobot, int *vectorX);

// TEST area defense
int computeArea(int angle);

// gestione caso palla dietro al portiere nel ritorno in porta 
void pallaDietro(struct data dataRobot); 

