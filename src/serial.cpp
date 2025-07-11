 #include "serial.h"
 #include "TeensyDebug.h"
 
 void MngSERIAL_Init(void) {
    // Initialize serial communication for debugging
    // This is the main serial port used for debugging and output
    // Uncomment the next line if you want to wait for Serial to be ready
    // Note: This is not necessary on Teensy boards as they initialize Serial automatically
 //while (!Serial);  // uncomment to have the sketch wait until Serial is ready

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Serial Debug Started");

// Debugger will use second USB Serial; this line is not need if using menu option
  debug.begin(SerialUSB1);

 }