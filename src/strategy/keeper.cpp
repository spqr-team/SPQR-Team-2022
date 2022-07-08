// keeper
#include "data/DataSensorsCompute.h"
#include "systems/degreesRemapping.h"
#include "bluetooth.h"
#include "strategy/keeper.h"
#include "motors/motors.h"
#include "motors/motoHolon.h"
#include "sensors/lines.h"
#include "sensors/ballRead.h"
#include "strategy/striker.h"
#include "PID_v2.h"
#include "vars.h"



// valori costanti di riferimento area di difesa portiere 
int AREA_CENTER = 0;
int ANG_CENTER = 0; 
int ANG_SX = 0;
int AREA_SX = 0;  
int ANG_DX = 0; 
int AREA_DX = 0; 



// timer catching the ball 
elapsedMillis tCatch = 0; 

// timer sensor NE
elapsedMillis timerSens = 0;  

// timer keeper attack 
elapsedMillis timerAttack; 


// init keeper coefficient and values
void init_keeper(bool attackGoal) {

    if(attackGoal == BLUE)  {
        ANG_CENTER = ANG_CENTER_Y;
        AREA_CENTER = AREA_CENTER_Y;
        ANG_DX = ANG_DX_Y;
        ANG_SX = ANG_SX_Y; 
    }
    else {
        ANG_CENTER = ANG_CENTER_B;
        AREA_CENTER = AREA_CENTER_B;
        ANG_DX = ANG_DX_B;
        ANG_SX = ANG_SX_B;
    }
}


// compute ideal goal's area 
int computeArea(int angle)  {

    int setpoint = 0; 

    if(angle >= ANG_CENTER) {
        setpoint = (AREA_DX - AREA_CENTER) / (ANG_DX - ANG_CENTER);
        setpoint = ((setpoint * (angle - ANG_DX)) + AREA_DX);
    }
    else {
        setpoint = (AREA_CENTER - AREA_SX) / (ANG_CENTER - ANG_SX);
        setpoint = ((setpoint * (angle - ANG_SX)) + AREA_SX);
    }

    if(setpoint < 0) setpoint = setpoint * (-1);
    setpoint = (int) setpoint;

    return setpoint; 
}


// compute keeper vector X (based on goal's angles)
int Keeper_vectX(struct data dataRobot, bool backToDefense, bool limits) {
    unsigned int deltaAng; 
    int angDefense;
    int angBall = dataRobot.angleBallC;
    int vectX = 0;
    static bool outSX = false; 
    static bool outDX = false;   


    // ball in front of the robot 
    if(angBall >= 350 || angBall <= 10) {
        vectX = 0; 
    }
    else if(angBall > 10  && angBall <= 180)  vectX = 1;    // right 
    else if(angBall < 350 && angBall > 180)   vectX = -1;   // left

    // velocity proportional
    if (angBall <= 180) deltaAng = angBall;
    else if(angBall > 180) deltaAng = 360 - angBall; 
    vectX = deltaAng * COEFF_VECT_X * vectX;

    // compute defense Goal angle 
    if(dataRobot.attackGoal == BLUE) angDefense = dataRobot.angYellowC;
    else angDefense = dataRobot.angBlueC;

    // ritorno in porta 
    if(backToDefense == true) {
        vectX = (angDefense - ANG_CENTER) * (-20);
        if(abs(angDefense - ANG_CENTER) < 10) vectX = 0; 
    }
    
    // fix out of limits
    if(limits == true) {
        if(angDefense > ANG_DX) {
            vectX = -600; 
            outDX = true; 
        }
        else if(angDefense < ANG_SX) {
            vectX = 600;
            outSX = true;   
        }
        else {
            if(outDX) {
                if((abs(angDefense - ANG_DX) < 15)  &&  (vectX > 0)) vectX = 0; 
                else outDX = false;  
            }
            else if(outSX) {
                if((abs(angDefense - ANG_SX) < 15)  &&  (vectX < 0)) vectX = 0;
                else outSX = false; 
            }
        }
    }    

    return vectX;
}



// compute keeper direction and velocity (when ball is far away)
void Keeper_compute(struct data *dataRobot, int *dir, int *vel, int *orient, bool back, bool toCenter) {
    
    double direction = 0; 
    int velocity = 0;
    int angle = 0; 
    byte zoneKeeper; 
    int maxVel = SIDES_VEL_KEEPER;
    int defense = dataRobot->defenseAng;
    byte Stotal = dataRobot->lineSensors_current;
    static bool left = false; 
    static bool right = false; 
    
   
    // compute zoneKeeper
    if(defense > (ANG_CENTER + WRAP_ANGLE) || (defense < (ANG_CENTER - WRAP_ANGLE)))    zoneKeeper = SIDES; 
    else zoneKeeper = CENTER; 


    // orient  
    if(zoneKeeper == SIDES) { // ai lati 
        angle = (int)((*dataRobot).defenseAng + 180 + (180 - ANG_CENTER)) % 360;
         *orient = angle; 
    } 
    else { // al centro 
        *orient = 0; 
    }
    
    // compute vectors
    int vectX = 0; 
    int vectY = 0; 

    // ritorno in porta 
    if(back == true) {  
        vectX = Keeper_vectX(*dataRobot, true, true); 
        vectY = Keeper_vectY(*dataRobot);
        *orient = 0; 
        maxVel = VEL_BACK_DEFENSE;  
    }
    else { // portiere normale 
        
        // vai al centro porta 
        if(toCenter == true)  vectX = Keeper_vectX(*dataRobot, true, true);
        else vectX = Keeper_vectX(*dataRobot, false, true);

        vectY = keeper_vectY_lines(*dataRobot);

        // fix vai più indietro ai lati 
        if(zoneKeeper == SIDES) {
            if(vectY < 0) vectY = vectY - 1000;  // prec = 100 
        }    
        else { // vai più veloce al centro davanti l'area 
            maxVel = CENTER_VEL_KEEPER; 
        }
    }
    

    // stop
    if(vectX == 0 && vectY == 0) maxVel = 0;

    // compute direction 
    direction = atan2((double)vectY, (double)vectX);
    *dir = ToDegree(direction);
    *dir = ToRobotAngles(*dir);

    // compute velocity 
    velocity = (int) sqrt(vectX*vectX + vectY*vectY);
    if(velocity > maxVel) velocity = maxVel;
    else if(velocity < MIN_VEL_KEEPER) velocity = MIN_VEL_KEEPER;
    *vel = velocity;


    // SPAZZATA 
    static unsigned long timerSpazzata = millis();
    static unsigned long timerCatch = millis(); 
    static bool catchBall = false; 
    static long pause = PAUSA_SPAZZATA_CENTRO; 
    
    
    if(vectX == 0 && dataRobot->distanceBallC < MAX_BALL_DISTANCE && back == false && (dataRobot->angleBallC < 135 || dataRobot->angleBallC > 225))  {
        
        // pausa spazzata 
        if(zoneKeeper == CENTER) pause = PAUSA_SPAZZATA_CENTRO;
        else pause = PAUSA_SPAZZATA_LATI;

        if(((millis() - timerSpazzata) > DELAY_SPAZZATA) && (catchBall == false) && ((millis() - timerCatch) > pause)) {
            catchBall = true; 
            tCatch = 0; 
        }

    } 
    else {
        timerSpazzata = millis(); 
    }

    // reset 
    // --- TEST non entrare nell'area durante la spazzata laterale ----
    if(tCatch > TIME_CATCH || NE) catchBall = false;

    // catching the ball
    if(catchBall == true)   {
        *dir = dataRobot->angleBallC;
        *vel = VELOCITY_CATCH; 
        timerCatch = millis();
    }

    // slow down after cathing the ball
    if((catchBall == false) && (millis() - timerCatch) > 100)  {
        if((*dir > 175) && (*dir < 185) && (*vel > 30)) *vel = 25;
    }

    // fix palla dietro: se palla vicina, sto tornando in porta e la palla è dietro di me 
    if(dataRobot->ballDist_thresholds == VERY_CLOSE || dataRobot->ballDist_thresholds == CLOSE)  {
        if(dataRobot->angleBallC > 150 && dataRobot->angleBallC < 210)  {
            if(right == false && left == false) {
                if(dataRobot->angleBallC >= 180)   right = true; 
                else left = true; 
            }

            if(right == true)       *dir = 110; 
            else if(left == true)   *dir = 250;

            if(Stotal > 0) *vel = 0; 
        }
        else {
            left = false; 
            right = false; 
        }
    }
    else {
        right = false; 
        left = false; 
    }


}



// keeper vector Y with defense area lines
int keeper_vectY_lines(struct data dataRobot)  {
    
    byte Stotal = dataRobot.lineSensors_current;
    static bool flag_out = false; 
    int vectY = 0; 

    
    // se non vedo la linea 
    // indietro veloce  
    if(Stotal == 0) {   
        vectY = -70;
    }
    // indietro 
    if(SE) { 
        vectY = -30;  
    }
    // stop
    if(SI) {  
        vectY = 0;
    }
    // avanti
    if(EI || OI) { // EI, EE, OI, OE OR tra tutti 
        vectY = 0; 
        vectY = 40; // prec = 50 
    }
    // casi speciali transizione orient 
    if(SI && SE) vectY = -20;  
 
    // LINEE non entra nell'area 
    if(NI) vectY = 2000; 
    // FIX ritorno in porta
    if(NE) { 
        vectY = 2000;
        timerSens = 0; 
    }

    // se entro completamente nell'area attivo flag
    if(timerSens < 50 && Stotal == 0) flag_out = true;  // prec millisecondi = 100
    
    // flag reset
    if(flag_out == true && Stotal != 0) flag_out = false; 

    // se entrato completamente eco fino a che non vedo la linea 
    if(flag_out == true) vectY = 2000;   

    // test coeff_vect_Y sul vettore 
    vectY = vectY * COEFF_VECT_Y;

    return vectY; 
}





// compute keeper vector Y (based on blob area)
int Keeper_vectY(struct data dataRobot) {

    int area = 0;  
    int setpoint = 0;
    int vectY = 0;

    // area and angle defense goal
    area = dataRobot.defenseArea;
    setpoint = AREA_CENTER; 

    // vector module
    vectY = abs(setpoint - area) * (-1); 
    vectY = vectY * COEFF_VECT_Y;

    // FIX tolleranza
    if((abs(setpoint - area)) < 40) vectY = 0; 
    
    return vectY;
}

