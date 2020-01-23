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
// individual fields will never exceed 20 bytes. Data
// stream starts approximately one minute after nano-labs are powered up
// Data stream will close approximately five minutes after landing. 

#include <SPI.h>  // required for SD library
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

#define CURR_GAIN_CONSTANT 68.4

// how long (in ms) it takes to prime the experiment
#define PRIME_TIME 1000

// how long (in ms) it takes to clean the experiment
#define CLEAN_TIME 1000

// data packet information
#define MAX_FRAME_SIZE 250
#define MAX_FIELD_SIZE 20
#define NUM_FIELDS 21
#define DELIMITER ','

// possible states for our cubesat
#define LS_NO_STATE 0
#define LS_IDLE 1
#define LS_PRIME_EXPERIMENT 2
#define LS_CELL_PLATING 3
#define LS_CLEAN_UP 4

// file names for logging, keeping track of state, etc.
#define LOG_FILE_PATH "log.txt"
#define STATE_FILE_PATH "state.txt"
#define DATA_FILE_PATH "data.txt"

#define LOG_TIME_CUTOFF 1000

// possible states for the blue rocket
#define BS_NO_STATE '@'
#define BS_ABORT_ENABLED 'A'
#define BS_ABORT_COMMANDED 'B'
#define BS_LIFTOFF 'C'
#define BS_MECO 'D'
#define BS_SEP_COMMANDED 'E'
#define BS_COAST_START 'F'
#define BS_APOGEE 'G'
#define BS_COAST_END 'H'
#define BS_DROGUE_DEPLOY 'I'
#define BS_MAIN_CHUTE_DEPLOY 'J'
#define BS_LANDING 'K'
#define BS_SAFING 'L'
#define BS_MISSION_END 'M'


// typedefs

// encapsulates environment data
typedef struct env_data_st {
  float volt_data;
  float curr_data;
  float temp_data;
} EnvData, *EnvDataPtr;

// encapsulates state information
typedef struct state_st {
  float last_blue_time;
  char lab_state;
  char blue_state;
  char last_state;
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
// measurements and stores them in the EnvData
// struct at EnvDataPtr
void read_sensors(EnvDataPtr env_data_ptr);

// writes data to data_file in comma-separated
// format
void log_sensor_data(EnvDataPtr env_data_ptr, File data_file);

// returns a time stamp formatted MM:SS:MSMS.
// size 'str' buffer
char* get_time_stamp(char* buf);

//// send a message to the active logging system
//// (either the serial line, or the log file)
//void log_msg(char* msg);

// records the state to a state file
// and prints to the active logging system
// that a state transition has occurred
void record_state();

// restores the state from the state file
void restore_state();

// end function prototypes


// global variables

// stores state information
State state;

// has the experiment been primed?
bool experiment_primed = false;

// has the experiment started?
bool experiment_started = false;

// have we finished cleaning the cell?
bool cleaning_finished = false;

<<<<<<< HEAD
// have we started priming?
bool priming_started = false;

// when did we start pumping?
long pump_start_time;

// have we started plating?
bool plating_started = false;

// when was the last time we took a measurement?
long last_log_time;

// have we started cleaning the experiment?
bool cleaning_started;

// when did we start cleaning
=======
// have we started priming the experiment?
bool priming_started;

// when did we start the pumps?
long pump_start_time;

// have we started plating?
bool plating_started;

// when did we last take a measurement?
long last_log_time;

// have we started cleaning?
bool cleaning_started;

// when did we start cleaning?
>>>>>>> state_rest_fix
long cleaning_start_time;

// global logging file
File log_file;

// global state file
File state_file;

// global data file
File data_file;

// end global variables


// debug macros
//#define DEBUG

#ifdef DEBUG
#define LOG_MSG(x) Serial.print(x)
#define LOG_MSG_LN(x) Serial.println(x)
#else
#define LOG_MSG(x)  log_file.print(x)
#define LOG_MSG_LN(x) log_file.println(x)
#endif

// end debug macros


// configures and initializes serial, sd,
// pump, and experiment interface
void setup() {
  // we want to ensure that we can log what's
  // happening with the cubesat. When we're
  // debugging then, output is reflected to the serial line
  // and thus we need it initialized first.
  #ifdef DEBUG
  serial_init();
  sd_init();
  #else
  sd_init();
  serial_init();
  #endif

  // determine if we need to read last state
  //if (SD.exists(STATE_FILE_PATH)) {
    // restore_state();
  //} else {
    // initialize default state
    LOG_MSG_LN("default state");
    state.last_blue_time = 0L;
    state.lab_state = LS_IDLE;
    state.blue_state = '@';
    state.last_state = LS_NO_STATE;
  //}
  
  // configure pins
  pin_init();
}

void serial_init() {
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);
  LOG_MSG_LN("Serial init");
  // this comment must go
}

void sd_init() {
  if (!SD.begin(CHIP_SELECT)) {
    LOG_MSG_LN("SD !init");  
  } else {
    log_file = SD.open(LOG_FILE_PATH, FILE_WRITE);
    LOG_MSG_LN("SD init");
  }
}

void pin_init() {
  pinMode(PUMP_POWER, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(EXPERIMENT, OUTPUT);

  // TODO: add solenoid pins

  // pins for sensor reading
  pinMode(TEMP_ANALOG_PIN, INPUT);
  pinMode(CURR_ANALOG_PIN, INPUT);
  pinMode(VOLT_ANALOG_PIN, INPUT);
  digitalWrite(EXPERIMENT, HIGH);

  LOG_MSG_LN("pins init");
}

// main state machine logic
void loop() {
<<<<<<< HEAD
  // check for serial data
=======
>>>>>>> state_rest_fix
  if (Serial.available() > 0) {
    read_serial_input();
  }
  
  switch(state.lab_state) {
    
    case LS_IDLE:
      {     
        // is it time to start priming?
        // check both no state and primed, since blue
        // may enter null state unexpectedly and we
        // want to know if we've already prepared the cell
        if (state.blue_state == BS_NO_STATE && !experiment_primed) {
          // TODO: log state transition to state file
          // and to log file
          LOG_MSG_LN("idle -> priming");
          priming_started = false;
          state.lab_state = LS_PRIME_EXPERIMENT;
          state.last_state = LS_IDLE;
        }

        // can we start the experiment?
        if (state.blue_state == BS_COAST_START) {
          // TODO: log state transition to state file
          // and to log file
          LOG_MSG_LN("idle -> plating");
          plating_started = false;
          state.lab_state = LS_CELL_PLATING;
          state.last_state = LS_IDLE;
        }

        // have we landed?
        if ((state.blue_state == BS_LANDING ||
                state.blue_state == BS_SAFING) &&
                !cleaning_finished) {
          // TODO: log state transition to state file
          // and to log file
          LOG_MSG_LN("idle -> clean up");
          cleaning_started = false;
          cleaning_finished = false;
          state.lab_state = LS_CLEAN_UP;
          state.last_state = LS_IDLE;
        }
      }
      break;
    case LS_PRIME_EXPERIMENT:
      {
        // is it time to start priming the experiment?
        if (!priming_started) {
          LOG_MSG_LN("priming");
          pump_start_time = millis();
          priming_started = true;
          // indicates that pumps are powered
          digitalWrite(PUMP_POWER, HIGH);
          digitalWrite(PUMP_1, HIGH);

          // TODO: add solenoid control logic
        }

        // if we've sufficiently primed the experiment,
        // we can stop priming and prepare to idle until
        // we start coasting
        if (millis() - pump_start_time >= PRIME_TIME) {
          LOG_MSG_LN("!priming");
          experiment_primed = true;
          digitalWrite(PUMP_POWER, LOW);
          digitalWrite(PUMP_1, LOW);

          // TODO: add solenoid control logic
          
          // TODO: log state transition to state file
          // and to log file
          LOG_MSG_LN("priming -> idle");
          state.lab_state = LS_IDLE;
          state.last_state = LS_PRIME_EXPERIMENT;
        }
      }
      break;
      
    case LS_CELL_PLATING:
      {
        if (!plating_started) {
          LOG_MSG_LN("plating");
          data_file = SD.open(DATA_FILE_PATH, FILE_WRITE);
          plating_started = true;
          digitalWrite(EXPERIMENT, LOW);
          last_log_time = millis();
        } else if (millis() - last_log_time >= LOG_TIME_CUTOFF) {
          // if enough time has passed, take a measurement
          // of the environment
          last_log_time = millis();
          EnvData env_data;
          read_sensors(&env_data);
          log_sensor_data(&env_data, data_file);
        }

        // if we're no longer in micro-gravity, 
        // terminate the experiment
        if (state.blue_state == BS_COAST_END) {
          data_file.close();
          LOG_MSG_LN("!plating");
          digitalWrite(EXPERIMENT, HIGH);
          // TODO: log state transition to state file
          // and to log file
          LOG_MSG_LN("plating -> idle");
          state.lab_state = LS_IDLE;
          state.last_state = LS_CELL_PLATING;
        }
      }
      break;

    case LS_CLEAN_UP:
      {
        // is it time to start cleaning up?
        // we'll leave the data and log files 
        // open until the very end
        if (!cleaning_started) {
          LOG_MSG_LN("cleaning");
          cleaning_start_time = millis();
          cleaning_started = true;
          digitalWrite(PUMP_POWER, HIGH);
          digitalWrite(PUMP_2, HIGH);
        } else if (millis() - cleaning_start_time >= CLEAN_TIME) {
          // we're finished cleaning and can close the files
          // and get ready again to idle
          LOG_MSG_LN("!cleaning");
          cleaning_finished = true;
          digitalWrite(PUMP_POWER, LOW);
          digitalWrite(PUMP_2, LOW);
          // TODO: log state transition to state file
          // and to log file
          LOG_MSG_LN("clean up -> idle");
          state.lab_state = LS_IDLE;
          state.last_state = LS_CLEAN_UP;
          log_file.close();
        }
      }
      break;
    case LS_NO_STATE:
      {
        // this is a null state and we should never be here
      }
    default:
      {
        // this is a null state and we should never be here  

        // log null state, and state transition
        // TODO: log state transition to state file
        // and to log file
        LOG_MSG_LN("null -> idle");
        state.lab_state = LS_IDLE;
        state.last_state = LS_NO_STATE;
      }
      break;
  }
}

void read_serial_input() {
  delay(20);
  int bytes_read = 0;
  char* fields[NUM_FIELDS];
  
  // there are 21 fields to read
  for (int i = 0; i < NUM_FIELDS; i++) {
    // plus 1 for null-terminator
    char buf[MAX_FIELD_SIZE + 1];
    int bytes = 0;
    char next = Serial.read();

    // each field is guaranteed to end in a comma
    // and be no more than 20 bytes in length
    while (next > -1 &&
              bytes <= MAX_FIELD_SIZE &&
              next != DELIMITER) {
      buf[bytes] = next;
      bytes++;
      next = Serial.read();
    }
    // terminate the field-string
    buf[bytes] = '\0';
    fields[i] = buf;
  }
  state.blue_state = fields[0][0];
  state.last_blue_time = atof(fields[1]);
}

void read_sensors(EnvDataPtr env_data_ptr) {
  env_data_ptr->curr_data = (float) analogRead(CURR_ANALOG_PIN) * (5.0 / 1023.0) / CURR_GAIN_CONSTANT;
  env_data_ptr->volt_data = 5.0 - (float) analogRead(VOLT_ANALOG_PIN) * (5.0 / 1023.0);
  env_data_ptr->temp_data = (float) analogRead(TEMP_ANALOG_PIN) * (5.0 / 1023.0) * 100;
}

void log_sensor_data(EnvDataPtr env_data_ptr, File data_file) {
  // convert sensor values to strings
  char s_volt[10];
  char s_curr[10];
  char s_temp[10];
  dtostrf(env_data_ptr->volt_data, 5, 3, s_volt);
  dtostrf(env_data_ptr->curr_data, 5, 3, s_curr);
  dtostrf(env_data_ptr->temp_data, 5, 3, s_temp);

  // write sensor data to data file
  #ifdef DEBUG
  LOG_MSG(millis());
  LOG_MSG(DELIMITER);
  LOG_MSG(s_volt);
  LOG_MSG(DELIMITER);
  LOG_MSG(s_curr);
  LOG_MSG(DELIMITER);
  LOG_MSG_LN(s_temp);
  #else
  data_file.print(millis());
  data_file.print(DELIMITER);
  data_file.print(s_volt);
  data_file.print(DELIMITER);
  data_file.print(s_curr);
  data_file.print(DELIMITER);
  data_file.println(s_temp);
  #endif
}

char* get_time_stamp(char* buf) {
  // extract components of time stamp
  unsigned long t = millis();
  unsigned long seconds = t / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long mils = t % 1000;
  seconds = seconds % 60;

  // convert extracted values to strings
  char s_min[4];
  char s_sec[3];
  char s_mils[5];
  ltoa(minutes, s_min, 10);
  ltoa(seconds, s_sec, 10);
  ltoa(mils, s_mils, 10);

  // concatenate output in str
  strcat(buf, "[");
  strcat(buf, s_min);
  strcat(buf, ":");
  strcat(buf, s_sec);
  strcat(buf, ":");
  strcat(buf, s_mils);
  strcat(buf, "]");
  return buf;
}
