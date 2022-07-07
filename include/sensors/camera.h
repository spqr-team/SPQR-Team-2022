// openMv h7 CAM

// no data on CAM serial buffer 
#define NO_DATA_AVAILABLE  666

// goal's blob not found
#define BLOB_NOT_FOUND  999  

// timer blob not found     
#define TIMER_BLOB_NOT_FOUND 5000

// coeff complementary filter goals area
#define COEFF_AREA_YELLOW 0.2 // il filtro reale sarà più veloce
#define COEFF_AREA_BLUE   0.2 // prec = 0.08



// read cama data
bool readCAM(int *blueGoal, int *yellowGoal, int *areaBlue, int *areaYellow, int orient);

// complementary filter CAM
void filterCAM(int *angBlu, int* angYellow);

// test CAM
void testCAM();
