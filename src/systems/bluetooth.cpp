// bluetooth communications and debug
// debug real-time wireless via bluetoth of teh robot, with hc-05.
// min delay to allow a proper communication with the app = 100ms
// protocol:  "data_name data_value\n"
// the baudrate for communications is 19200 for all BT modules

#include <Arduino.h>
#include "vars.h"
#include "Bluetooth.h"


// timeout BT communications
elapsedMillis timeoutRX;
elapsedMillis timeoutTX;


// debug data packet string to send 
static String data_packet = "";
static int data_counter = 0; 



void BTdebug_config() {
// echo in order to config bluetooth module hc-05 (master and slave)
// baudrate for bluetooth links = 19200 
// to enter in AT mode press the button and turn on the module 
// in AT mode you have to set the baudrate = 38400
    
    // init serial once 
    static bool init = false; 
    if(init == false) {
        BLUETOOTH.begin(38400);
        init = true; 
    }

    // char data 
    char c;

    // Read from the Bluetooth module and send to the Arduino Serial Monitor
    if (BLUETOOTH.available()) {
        c = BLUETOOTH.read();
        Serial.print((char)c);
    }
  
    // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available()) {
        c = Serial.read(); 
        BLUETOOTH.write(c);
    }
  
    // slow down the communication
    delay(30);
}



// check if the bluetooth link is connected (robots are on the field)
bool BTisConnected() {

    char check;

    // receive check 
    if(BLUETOOTH.available()) {
        check = BLUETOOTH.read();
        if(check == 'c')  timeoutRX = 0; 
    }

    // transmit check 
    if(timeoutTX >= 50)  {
        BLUETOOTH.print('c');
        timeoutTX = 0; 
    }

    if(timeoutRX < 600) return true; // connected
    else return false;               // disconnected
}





// -----DEBUG as slave---------
void BTdebug_addData(String newdata) {
    // add a new data to send over BTdebug
    if(data_counter <= 7) {
        newdata += "\n";
        data_packet += newdata;
        data_counter++;
    }
}
void BTdebug_send() {
    // send the entire packet of data to BT debugger every 100ms
    static unsigned long t = millis(); 

    if((millis() - t) >= 100) {
        BLUETOOTH.print(data_packet);
        data_packet = "";
        data_counter = 0; 
        t = millis(); 
    }

}
// -----DEBUG as slave--------




