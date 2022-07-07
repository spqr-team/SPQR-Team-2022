// bluetooth communications

#include <Arduino.h>


// add new data to debug packet
void BTdebug_addData(String newdata);

// send the debug packet 
void BTdebug_send();

// config the hc-05 bluetooth module (echo)
void BTdebug_config();

// check if the robots are connected
bool BTisConnected();
