// Eli Reed, November 18th 2019
// written for the NanoLab mission

// Controller for the electroplating experiment we're running
// for our WASGC-1 Mission (name to-be-changed hopefully).

// Blue Serial Connection Spec:
// Baud Rate: 115200
// Data: 8 bits |
// Parity: None |--> 8N1
// Stop Bits: 1 |
// 
// Packets are ASCII string of 21 comma-separated values
// size of packets <= 250 bytes (buffer is 256 bytes), 
// individual fields will never exceed 20 bytes Data
// stream starts approximately one minute after nano-labs are powered up

#include <SD.h>   // exposes functions for writing to/reading from SD card

// pin configuration macros
#define CHIP_SELECT 10
#define TEMP_ANALOG_PIN A0
#define CURR_ANALOG_PIN A1
#define VOLT_ANALOG_PIN A2

#define PUMP_POWER 4
#define PUMP_1 5
#define PUMP_2 6

#define EXPERIMENT 9

// data packet information
#define MAX_FRAME_SIZE 250
#define MAX_FIELD_SIZE 20

// possible states for our cubesat
#define LS_NO_STATE 0
#define LS_IDLE 1
#define LS_SERIAL_READ 2
#define LS_PRIME_EXPERIMENT 3
#define LS_CELL_PLATING 4
#define LS_CLEAN_CELL 5
#define LS_LOG 6

// file names for logging, keeping track of state, etc.
#define LOG_FILE "log.txt"
#define STATE_FILE "state.txt"
#define DATA_FILE "data.csv"

// possible states for the blue rocket
#define BS_NO_STATE "@"
#define BS_ABORT_ENABLED "A"
#define BS_ABORT_COMMANDED "B"
#define BS_LIFTOFF "C"
#define BS_MECO "D"
#define BS_SEP_COMMANDED "E"
#define BS_COAST_START "F"
#define BS_APOGEE "G"
#define BS_COAST_END "H"
#define BS_DROGUE_DEPLOY "I"
#define BS_MAIN_CHUTE_DEPLOY "J"
#define BS_LANDING "K"
#define BS_SAFING "L"
#define BS_MISSION_END "M"


// typedefs

// encapsulates environment data
typedef struct env_data_st {
  float volt;
  float curr;
  float temperature;
} EnvData, *EnvDataPtr;

// encapsulates state information
typedef struct state_st {
  uint8_t lab_state;
  uint8_t blue_state;
  uint8_t last_state;
  
} State;

// end typedefs


// function prototypes

// intializes serial interface
void serial_init();

// initializes SD card interface
void sd_init();

// configures pins on which pumps, 
// the experiment and the sensors are driven
void pin_init();

// takes current, voltage, and temperature
// measurements and stores them in EnvData
void read_sensors(EnvDataPtr env_data);

// returns a time stamp formatted MM:SS:MSMS
String get_time_stamp();



// end function prototypes

// global variables

// the current state the lab is in
int lab_state;

// the last seen state of the rocket
String blue_state;

// the previous state the lab was in
int last_lab_state;

// last MET received from rocket
float last_blue_time = 0;

// has the experiment been primed?
bool experiment_primed = false;

// for debugging
const bool DEBUG = true;

// end global variables


// configures and initializes serial, sd,
// pump, and experiment interface
void setup() {
  // initialize serial interface
  serial_init();

  // initialize SD interface
  sd_init();

//  // determine if we need to read last state
//  if (SD.exists(STATE_FILE)) {
//    // restore the previous state, and go from there
//  } else {
//    // configure default state
//  }
  
  // configure pins
  pin_init();
}

void serial_init() {
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);
  Serial.println("Serial initialized");
}

void sd_init() {
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("SD failed to initialize");  
  } else {
    Serial.println("SD initialized");
  }
}

void config_pins() {
  pinMode(PUMP_POWER, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(EXPERIMENT, OUTPUT);

  // pins for sensor reading
  pinMode(TEMP_ANALOG_PIN, INPUT);
  pinMode(CURR_ANALOG_PIN, INPUT);
  pinMode(VOLT_ANALOG_PIN, INPUT);
}

void loop() {
  switch(lab_state) {
    case LS_IDLE:
      {     
        if (Serial.available() > 0) {
          read_serial_input();
        }
      }
      break;
    case LS_SERIAL_READ:
      {
        
      }
      break;
      
    case LS_PRIME_EXPERIMENT:
      {

      }
      break;
      
    case LS_CELL_PLATING:
      {
        
      }
      break;

    case LS_CLEAN_CELL:
      {
        
      }
      break;
      
    case LS_LOG:
      { 
        
      }
      break;
    default:
      {
        
      }
      break;
  }
}

void read_serial_input() {
  delay(20);
  for (int i = 0; i < 21; i++) {
    char buf[21];
    int bytes = 0;
    char next = Serial.read();
    while (next > -1 && bytes < 21 && next != ',') {
      buf[bytes] = next;
      bytes++;
      next = Serial.read();
    }
//          for (bytes = 0; bytes <= 21; bytes++) {
//            if (Serial.peek() > -1) {
//              char next = Serial.read();
//              if (next == ',') {
//                break;
//              } else {
//                buf[bytes] = next;
//              }
//            } else {
//              break;
//            }
//          }
    buf[bytes] = '\0';
    if (i == 0) {
      blue_state = buf;
    }
    if (i == 1) {
      last_blue_time = ((String)buf).toFloat();
    }
  }
  lab_state = last_lab_state;
  last_lab_state = LS_SERIAL_READ;
  lf.close();
}

String get_time_stamp() {
  unsigned long t = millis();
  unsigned long seconds = t / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long mils = t % 1000;
  seconds = seconds % 60;
  String s_min = String(minutes);
  String s_sec = String(seconds);
  String s_mils = String(mils);
  return "[" + s_min + ":" + s_sec + ":" + s_mils + "]";
}
