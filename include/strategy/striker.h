#define ANG_TRAIETTORIA 45      // angolo di sfasamento 
#define DIST_CLOSE     117      // valore soglia distanza
#define COEFF_CLOSE    2.3      // coeff palla vicina     
#define COEFF_FAR      1.8      // coeff palla lontana 
#define COEFF_GOALIE   2.2      // coeff sfasamento 

// goalie
void attaccante(struct data dataRobot, int * dir);

// storcimento 
void orientToGoal(struct data dataRobot,int * orient);

// init attack goal 
void initAttackGoal();

// strategy with kicker 
void kick_strategy(struct data dataRobot);
