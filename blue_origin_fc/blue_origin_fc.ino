// Eli Reed (sailedeer)
// November 18th, 2019
// 
// Flight controller for MiniMEE mission. Maintains state of
// payload, facilitates communication with New Shepard,
// and controls experimentation. 

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
#define CHIP_SELECT A0
#define TEMP_ANALOG_PIN A1
#define CURR_ANALOG_PIN A3
#define VOLT_ANALOG_PIN A2

#define PUMP_POWER 2
#define PUMP_1 5
#define PUMP_2 6

#define SOL_1 8
#define SOL_2 9
#define SOL_3 10

#define MOTOR 3

#define EXPERIMENT A5

// Sensor gain constants
#define CURR_GAIN_CONSTANT 68.4

// how long (in ms) it takes to prime the experiment
#define PRIME_TIME 1000

// how long (in ms) to wait before priming
#define PRIME_WAIT_TIME 1000

// how long (in ms) it takes to finish stage 1
#define STAGE_1_LENGTH 1000

// how long (in ms) it takes to finish stage 2
#define STAGE_2_LENGTH 1000

// how long to wait before taking another measurement
#define LOG_TIME_CUTOFF 500

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

// bit masks for stage
#define LS_NO_STATE      0x0000
#define LS_IDLING_M      0x0001  // 1 << 0
#define LS_PRIMING_M     0x0002  // 1 << 1
#define LS_PRIMED_M      0x0004  // 1 << 2
#define LS_PLATING_M     0x0008  // 1 << 3, implies PRIMED
#define LS_PLATED_M      0x0010  // 1 << 4, implies PRIMED
#define LS_CLEANING_1_M  0x0020  // 1 << 5, implies PRIMED, PLATED
#define LS_CLEANING_2_M  0x0040  // 1 << 6, implies PRIMED, PLATED
#define LS_CLEANING_3_M  0x0080  // 1 << 7, implies PRIMED, PLATED
#define LS_CLEANING_4_M  0x0100  // 1 << 8, implies PRIMED, PLATED
#define LS_CLEANING_5_M  0x0200  // 1 << 9, implies PRIMED, PLATED
#define LS_CLEANED_M     0x0400  // 1 << 10, implies PRIMED, PLATED, CLEANING 1-5

#define MAGIC_NUMBER 0x451b

#define V_REF 1.1

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
  uint16_t lab_state;
  char blue_state;
  char last_blue_state;
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
void log_sensor_data(const EnvDataPtr env_data_ptr, File data_file);

// log message to logging system
void log_msg(const float t, const char* msg);

// records the state to a state file
// and prints to the active logging system
// that a state transition has occurred
void record_state();

// restores the state from the state file
void restore_state();

// end function prototypes


// global variables

// contains information about the flight state of MiniMEE
// and New Shepard
State state;

// acts as record of events during flight
File log_file;

// stores environmental/experimental data
File data_file;

// end global variables


// debug macros

#define DEBUG

#ifdef DEBUG
#define LOG_MSG(x) Serial.print(x)
#define LOG_MSG_LN(x) Serial.println(x)
#else
#define LOG_MSG(x) log_file.print(x)
#define LOG_MSG_LN(x) log_file.println(x)
#endif

// end debug macros

// initialize serial interface
void serial_init() {
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);
  Serial.setTimeout(20);  // ensures that we capture every packet
  log_msg(0.0f, "Serial");
}

// initialize SD card interface
void sd_init() {
  if (!SD.begin(CHIP_SELECT)) {
    log_msg(0.0f, "!SD");  
  } else {
    log_file = SD.open(LOG_FILE_PATH, FILE_WRITE);
    log_msg(0.0f, "SD");
  }
}

// initialize pin 
void pin_init() {
  // nominally, this should be 1.1V
  analogReference(INTERNAL);

  // initialize pump pins
  pinMode(PUMP_POWER, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);

  // initialize experiment pin
  pinMode(EXPERIMENT, OUTPUT);

  // initialize motor pin
  pinMode(MOTOR, OUTPUT);

  // initialize solenoid pins
  pinMode(SOL_1, OUTPUT);
  pinMode(SOL_2, OUTPUT);
  pinMode(SOL_3, OUTPUT);

  // pins for sensor reading
  pinMode(TEMP_ANALOG_PIN, INPUT);
  pinMode(CURR_ANALOG_PIN, INPUT);
  pinMode(VOLT_ANALOG_PIN, INPUT);

  // valves are active low
  // pumps are active high
  // pump power active low
  // motor active low
  // experiment is active low
  // TODO: active state of motor?
  digitalWrite(PUMP_POWER, HIGH);
  digitalWrite(PUMP_1, LOW);
  digitalWrite(PUMP_2, LOW);
  digitalWrite(SOL_1, HIGH);
  digitalWrite(SOL_2, HIGH);
  digitalWrite(SOL_3, HIGH);
  digitalWrite(MOTOR, HIGH);
  digitalWrite(EXPERIMENT, HIGH);

  log_msg(state.last_blue_time, "pins");
}

// reads state information from state file
// and attempts to re-initialize the lab to that state. 
void restore_state() {
  #ifndef DEBUG
  File state_file = SD.open(STATE_FILE_PATH, FILE_READ);
  if (state_file.peek() > -1) {
    // read state data
    state.lab_state = state_file.read();
    state_file.read();  // consume delimiter
    state.last_state = state_file.read();
    state_file.read();  // consume delimiter
    state.blue_state = state_file.read();
  } else {
    LOG_MSG_LN("!state_data");
  }
  state_file.close();
  SD.remove(STATE_FILE_PATH);
  #endif
}

// writes current lab state to state file
void record_state() {
  #ifndef DEBUG
  if (SD.exists(STATE_FILE_PATH)) {
    SD.remove(STATE_FILE_PATH);
  }
  // record:
  //  magic number (first 2 bytes)
  //  last blue time (precisely 4 bytes (float))
  //  last blue state (1 byte (ASCII char)
  //  last lab state (1 byte (between 1 and 11)

  File state_file = SD.open(STATE_FILE_PATH, FILE_WRITE);
  // write dummy 2 bytes
  state_file.print((uint16_t) 0);

  // begin writing real state data
  state_file.print(state.lab_state);
  state_file.print(DELIMITER);
  state_file.print(state.last_state);
  state_file.print(DELIMITER);
  state_file.print(state.blue_state);
  state_file.println(DELIMITER);
  state_file.close();
  #endif
}

// attempts to read a new serial packet from the 
// featherframe serial interface. 
void read_serial_input() {
  // according to NRFF Toolbox, need to wait
  // 17 ms for frame to finish transmission
  int fields_read = 0;
  char *fields[NUM_FIELDS];
  char buf[MAX_FRAME_SIZE + 1];
  size_t bytes = Serial.readBytes(buf, MAX_FRAME_SIZE);
  buf[bytes] = '\0';

  // process the frame
  char * ptr;
  ptr = strtok(buf, ",");
  int i = 0;
  while (ptr != NULL) {
    fields[i] = ptr;
    ptr = strtok(NULL, ",");
    i++;
  }

  // extract what we care about
  state.blue_state = fields[0][0];
  state.last_blue_time = atof(fields[1]);
  // TODO: expand this list?
}

// poll sensors for current environmental data. Stores results
// in pointer to an EnvData struct
void read_sensors(EnvDataPtr env_data_ptr) {
  env_data_ptr->curr_data = (float) analogRead(CURR_ANALOG_PIN) * (V_REF / 1023.0) / CURR_GAIN_CONSTANT;
  env_data_ptr->volt_data = V_REF - (float) analogRead(VOLT_ANALOG_PIN) * (V_REF / 1023.0);
  env_data_ptr->temp_data = (float) analogRead(TEMP_ANALOG_PIN) * (V_REF / 1023.0) * 100;
}

// write polled sensor data to file
void log_sensor_data(const EnvDataPtr env_data_ptr, File data_file) {
  // convert sensor values to strings
  char s_volt[10];
  char s_curr[10];
  char s_temp[10];
  dtostrf(env_data_ptr->volt_data, 5, 3, s_volt);
  dtostrf(env_data_ptr->curr_data, 5, 3, s_curr);
  dtostrf(env_data_ptr->temp_data, 5, 3, s_temp);

  // write sensor data to data file
  #ifdef DEBUG
  LOG_MSG(state.last_blue_time);
  LOG_MSG(DELIMITER);
  LOG_MSG(s_volt);
  LOG_MSG(DELIMITER);
  LOG_MSG(s_curr);
  LOG_MSG(DELIMITER);
  LOG_MSG_LN(s_temp);
  #else
  data_file.print(state.last_blue_time);
  data_file.print(DELIMITER);
  data_file.print(s_volt);
  data_file.print(DELIMITER);
  data_file.print(s_curr);
  data_file.print(DELIMITER);
  data_file.println(s_temp);
  #endif
}

// write a message to the log file at time t
void log_msg(const float t, const char* msg) {
  LOG_MSG(t);
  LOG_MSG(DELIMITER);
  LOG_MSG_LN(msg);
}

// check if the bits specified by mask are flipped
bool check_lab_state(const uint16_t &mask) {
  return (state.lab_state & mask);
}

// start the first cleaning step
void start_cleaning_1() {
  log_msg(state.last_blue_time, "clean1");
  digitalWrite(SOL_2, LOW);
  digitalWrite(SOL_3, LOW);
                 
  // run p2
  digitalWrite(PUMP_POWER, LOW);
  digitalWrite(PUMP_2, HIGH);
}

// stop the first cleaning step
void stop_cleaning_1() {
  digitalWrite(SOL_2, HIGH);
  digitalWrite(SOL_3, HIGH);
                 
  // run p2
  digitalWrite(PUMP_POWER, HIGH);
  digitalWrite(PUMP_2, LOW);
}

// start the second cleaning step
void start_cleaning_2() {
  log_msg(state.last_blue_time, "clean2");
  digitalWrite(SOL_1, LOW);
  digitalWrite(SOL_3, LOW);

  // run p1
  digitalWrite(PUMP_POWER, LOW);
  digitalWrite(PUMP_1, HIGH);
}

// stop the second cleaning step
void stop_cleaning_2() {
  digitalWrite(SOL_1, HIGH);
  digitalWrite(SOL_3, HIGH);

  // run p1
  digitalWrite(PUMP_POWER, HIGH);
  digitalWrite(PUMP_1, LOW);
}

// configures and initializes serial, sd,
// pump, solenoid, experiment interfaces
void setup() {
  // need serial line configured first if debugging
  #ifdef DEBUG
  serial_init();
  sd_init();
  #else
  sd_init();
  serial_init();
  #endif

  // determine if we need to read last state
  if (SD.exists(STATE_FILE_PATH)) {
    restore_state();
    log_msg(state.last_blue_time, "hot");
  } else {
    // initialize default state
    state.last_blue_time = 0.0f;
    state.lab_state  = LS_NO_STATE;
    state.blue_state = BS_NO_STATE;
    state.last_blue_state = BS_NO_STATE;
    log_msg(state.last_blue_time, "cold");
  }
  
  // configure pins
  pin_init();
}

// state machine predicated on state of blue rocket
void loop() {
  if (Serial.available() > 0) {
    read_serial_input();
  }

  if (state.blue_state != state.last_blue_state) {
    record_state();
  }

  switch(state.blue_state) {

    case BS_SEP_COMMANDED:    // CC is free
    {
      // have we already primed?
      if (!check_lab_state(LS_PRIMED_M)) {
        // wait until things have settled down
        static unsigned long sep_cmd_time = millis();
        if (millis() - sep_cmd_time < PRIME_WAIT_TIME) {
          break;
        }

        static unsigned long prime_start_time;
        if (!check_lab_state(LS_PRIMING_M)) {
          // start priming
          state.lab_state &= ~LS_IDLING_M;
          state.lab_state |= LS_PRIMING_M;
          record_state();

          digitalWrite(MOTOR, LOW);

          log_msg(state.last_blue_time, "priming");

          prime_start_time = millis();
        } else if (millis() - prime_start_time >= PRIME_TIME) {
          // stop priming
          state.lab_state &= ~LS_PRIMING_M;
          state.lab_state |= (LS_PRIMED_M | LS_IDLING_M);
          record_state();

          digitalWrite(MOTOR, HIGH);

          log_msg(state.last_blue_time, "!priming");
        }
      }
    }
    break;

    case BS_APOGEE:
    {
      // do nothing, we want to flow into coast start
      // since we should still be plating at this point
    }
    
    case BS_COAST_START:
    {
      // if primed, start experiment
      // take a measurement (every .25 seconds)
      if (check_lab_state(LS_PRIMED_M)) {
        static unsigned long logging_time;
        if (!check_lab_state(LS_PLATING_M)) {
          state.lab_state &= ~LS_IDLING_M;
          state.lab_state |= LS_PLATING_M;
          digitalWrite(EXPERIMENT, LOW);
          
          data_file = SD.open(DATA_FILE_PATH, FILE_WRITE);
  
          logging_time = millis();
        } else if (millis() - logging_time >= LOG_TIME_CUTOFF) {
          // take a measurement
          EnvData env_data;
          read_sensors(&env_data);
          log_sensor_data(&env_data, data_file);
          logging_time = millis();
        }
      } else {
        // TODO: modularize priming
      }
    }
    break;

    case BS_COAST_END:
    {
      if (!(state.lab_state & LS_PLATED_M)) {
        state.lab_state |= LS_IDLING_M | LS_PLATED_M;
        state.lab_state &= ~LS_PLATING_M;
        record_state();
        
        log_msg(state.last_blue_time, "!plating");
        log_msg(state.last_blue_time, "plating->idle");
        
        // clean up
        data_file.close();
        digitalWrite(EXPERIMENT, HIGH);
      }
    }
    break;

    case BS_LANDING:
    {
      // flow into safing state since we do the same thing
    }

    case BS_SAFING:   
    {
      if (!check_lab_state(LS_CLEANED_M)) {
        static unsigned long clean_time;
        if (check_lab_state(LS_CLEANING_1_M)) {
          // transition to 2
          if (millis() - clean_time >= STAGE_1_LENGTH) {
            state.lab_state &= ~LS_CLEANING_1_M;
            state.lab_state |= LS_CLEANING_2_M;
            stop_cleaning_1();
            start_cleaning_2();
            clean_time = millis();
          }
          // cleaning_step_1();
        } else if (check_lab_state(LS_CLEANING_2_M)) {
          // transition to 3
          if (millis() - clean_time >= STAGE_2_LENGTH) {
            state.lab_state &= ~LS_CLEANING_2_M;
            state.lab_state |= LS_CLEANING_3_M;
            stop_cleaning_2();
            start_cleaning_1();
            clean_time = millis();
          }
        } else if (check_lab_state(LS_CLEANING_3_M)) {
          // transition to 4
          if (millis() - clean_time >= STAGE_1_LENGTH) {
            state.lab_state &= ~LS_CLEANING_3_M;
            state.lab_state |= LS_CLEANING_4_M;
            stop_cleaning_1();
            start_cleaning_2();
            clean_time = millis();
          }
        } else if (check_lab_state(LS_CLEANING_4_M)) {
          // transition to 5
          if (millis() - clean_time >= STAGE_2_LENGTH) {
            state.lab_state &= ~LS_CLEANING_4_M;
            state.lab_state |= LS_CLEANING_5_M;
            stop_cleaning_2();
            start_cleaning_1();
            clean_time = millis();
          }
        } else if (check_lab_state(LS_CLEANING_5_M)) {
          // end
          if (millis() - clean_time >= STAGE_1_LENGTH) {
            state.lab_state &= ~LS_CLEANING_5_M;
            state.lab_state |= LS_CLEANED_M | LS_IDLING_M;
            stop_cleaning_1();
            record_state();

            // clean up whole lab
            log_msg(state.last_blue_time, "!cleaning");
            
            log_msg(state.last_blue_time, "clean->idle");
  
            // close streams
            log_file.close();
            SD.remove(STATE_FILE_PATH);
          }
        } else {
          // we haven't done any cleaning yet
          state.lab_state &= ~LS_IDLING_M;
          state.lab_state |= LS_CLEANING_1_M;
          start_cleaning_1();
          clean_time = millis();
        }
      }
    }
    break;

    case BS_NO_STATE:   // on the pad
    {
      // simply flow into default state
    }

    default:            // every other state we don't really care about (MECO, etc)
    {
      if (!check_lab_state(LS_IDLING_M)) {
        state.lab_state |= LS_IDLING_M;
        record_state();
      }
    }
    break;
  }

  // assign last state now so that we can capture
  // any updates to state on next loop
  state.last_blue_state = state.blue_state;
}
