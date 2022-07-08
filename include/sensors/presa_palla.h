#define PIN_PRESA_PALLA    A13
#define SOGLIA_PRESA_PALLA 100 // prec lab = 200 // casa: 156 // filtro IR: 100 
#define COEFF_PRESA_PALLA_FILTER 0.01 // prec = 0.01

// se ho preso la palla in bocca
bool presaPalla(struct data dataRobot);

// test presa palla 
void test_presaPalla();
