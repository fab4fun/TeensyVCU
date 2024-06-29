/*
 * Simple data logger.
 */
#include <SPI.h>
#include "SdFat.h"

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// Interval between data records in milliseconds.
// The interval must be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 1000;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
// File system object.
extern SdFat sd;

// Log file.
extern SdFile file;

// Time in micros for next data record.
extern uint32_t logTime;

//==============================================================================
// User functions.  Edit writeHeader() and logData() for your requirements.

const uint8_t ANALOG_COUNT = 4;

//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))

void SD_setup(void);
void logData(void);
void writeHeader(void);

