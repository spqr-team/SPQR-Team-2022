// position system of the robot 
#define COORD_NOT_FOUND 300

// coefficient complementary filter on distance from the robot to a point
#define DISTANCE_POINT_FILTER 0.1

// coeffcient GOTOCENTER velocity
#define COEFF_VEL_GOTOCENTER 4

// filter goal angles postion system 
#define COEFF_GOAL_POSITION 0.5

// vel gotoCenter() lontano
#define VEL_GOTOCENTER_MAX  50

// vel gotoCenter() vicino
#define VEL_GOTOCENTER_MIN  30

// vel gotoPoint() 
#define VEL_GOTOPOINT_MAX  60

// center position coordinates 
#define UNDEFINED_COORD 111


// compute position
void computeCoords(int angB, int angY, bool attack, int * coordX, int * coordY);

// go to point (x, y)
int GoToCoords(int coordX, int coordY, int robotX, int robotY);

// compute distance between the robot and a point (x,y)
unsigned int computeDistance(int coordX, int coordY, int robotX, int robotY);

//  gotoCenter() of the field 
void gotoCenter(struct data dataRobot, int *dir, int *vel, int *orient);

// go to a point in the field 
bool goToPoint(struct data dataRobot, int pointX, int pointY, int *dir, int *vel);




// complementary filter goal angles
int filterAngCAM_blue(int ang);
int filterAngCAM_yellow(int ang);
