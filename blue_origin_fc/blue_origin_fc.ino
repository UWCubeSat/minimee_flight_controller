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
#define PRIME_TIME 2000

// how long (in ms) to wait before priming
#define PRIME_WAIT_TIME 5000

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

uint8_t cleaning_stage(uint16_t curr_clean_stage, uint16_t next_clean_stage);

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
    state.lab_state = LS_IDLING_M;
    state.blue_state = BS_NO_STATE;
    state.last_blue_state = BS_NO_STATE;
    log_msg(state.last_blue_time, "cold");
  }
  
  // configure pins
  pin_init();

  analogReference(INTERNAL);
}

void serial_init() {
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);    // might want a timeout here
  log_msg(0.0f, "Serial");
}

void sd_init() {
  if (!SD.begin(CHIP_SELECT)) {
    log_msg(0.0f, "!SD");  
  } else {
    log_file = SD.open(LOG_FILE_PATH, FILE_WRITE);
    log_msg(0.0f, "SD");
  }
}

void pin_init() {
  // analogReference(INTERNAL);
  
  pinMode(PUMP_POWER, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(EXPERIMENT, OUTPUT);

  // pin for controlling motor
  pinMode(MOTOR, OUTPUT);

  // pins for solenoids
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
  digitalWrite(SOL_1, HIGH);
  digitalWrite(SOL_2, HIGH);
  digitalWrite(SOL_3, HIGH);
  digitalWrite(MOTOR, HIGH);
  digitalWrite(EXPERIMENT, HIGH);

  log_msg(state.last_blue_time, "pins");
}

// main state machine logic
void loop() {
  // time serial processing
  //unsigned long t = millis();
  
  // loop() specific timer
  // declared static so timer waits in states
  // are 'non-blocking'
  static unsigned long loop_time;
  
  if (Serial.available() > 0) {
    read_serial_input();
  }

  if (state.blue_state != state.last_blue_state) {
    // this means we've transitioned state,
    // so we need to record
    // record_state();
  }

  // Serial.println(state.blue_state);

  switch(state.blue_state) {

    case BS_SEP_COMMANDED:    // CC is free
    {
      // (1) wait until rockets are done firing
      
      // timers for state are:
      // prime time
      // engine firing time

      log_msg(0.0, "sep_cmd");

      // engine firing timer
      loop_time = millis();
      
      if (millis() - loop_time >= PRIME_WAIT_TIME && 
          !(state.lab_state & LS_PRIMING_M)) {
        // set priming bit and record state
        state.lab_state &= ~LS_IDLING_M;
        state.lab_state |= LS_PRIMING_M;
        record_state();

        // engage the motor and start timer
        digitalWrite(MOTOR, LOW);

        // priming timer
        loop_time = millis();

        log_msg(state.last_blue_time, "priming");
      }

      // (2) wait until priming finished (assuming it has started)
      if ((state.lab_state & LS_PRIMING_M) &&
        millis() - loop_time >= PRIME_TIME) {
        // stop priming
        state.lab_state &= ~LS_PRIMING_M;
        state.lab_state |= LS_PRIMED_M;
        record_state();

        // disengage the motor
        digitalWrite(MOTOR, HIGH);

        log_msg(state.last_blue_time, "!priming");
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
      if ((state.lab_state & LS_PRIMED_M) &&
          !(state.lab_state & LS_PLATING_M)) {
        digitalWrite(EXPERIMENT, LOW);
        
        data_file = SD.open(DATA_FILE_PATH, FILE_WRITE);

        // data logging timer
        loop_time = millis();
      } else if ((millis() - loop_time >= LOG_TIME_CUTOFF) &&
                 (state.lab_state & LS_PLATING_M)) {
        loop_time = millis();
        EnvData env_data;
        read_sensors(&env_data);
        log_sensor_data(&env_data, data_file);
      } else {
        // we need to prime the experiment
        // TODO: Modularize priming
      }
    }
    break;

    case BS_COAST_END:
    {
      if (!(state.lab_state & LS_PLATED_M)) {
//      state.last_state = LS_CELL_PLATING;
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
      log_msg(state.last_blue_time, "cleaning");
      static uint8_t stage = 1;
      static uint8_t step = 1;
      static bool stage_started = false;
      
////      LOG_MSG_LN(step);
      if (stage == 1) {
        if (!stage_started) {
          if (step == 3) {
            state.lab_state |= LS_CLEANING_3_M;
            state.lab_state &= ~LS_CLEANING_2_M;
          } else if (step == 5) {
            state.lab_state |= LS_CLEANING_5_M;
            state.lab_state &= ~LS_CLEANING_4_M;
          }
          record_state();
          log_msg(state.last_blue_time, "clean_stage_1");
          digitalWrite(SOL_2, LOW);
          digitalWrite(SOL_3, LOW);
                         
          // run p2
          digitalWrite(PUMP_POWER, LOW);
          digitalWrite(PUMP_2, HIGH);
          loop_time = millis();
          stage_started = true;
        }
        
        if (millis() - loop_time >= STAGE_1_LENGTH) {
          loop_time = 0L;
          
          // turn everything off
          digitalWrite(PUMP_POWER, HIGH);
          digitalWrite(PUMP_2, LOW);
//          LOG_MSG_LN("disabling valves 2 and 3");
          digitalWrite(SOL_2, HIGH);
          digitalWrite(SOL_3, HIGH);
          if (step == 5) {
            // update and record state
            state.lab_state &= ~LS_CLEANING_5_M;
            state.lab_state |= LS_CLEANED_M | LS_IDLING_M;
//            state.lab_state = LS_IDLE;
//            state.last_state = LS_CLEAN_UP;
            record_state();
            
            log_msg(state.last_blue_time, "!cleaning");
            
            log_msg(state.last_blue_time, "clean->idle");
  
            // close streams
            log_file.close();
            SD.remove(STATE_FILE_PATH);
          } else {
            stage = 2;
            step++;
            stage_started = false;
          }
        }
      } else if (stage == 2) {
        if (!stage_started) {
          if (step == 2) {
            state.lab_state &= ~LS_CLEANING_1_M;
            state.lab_state |= LS_CLEANING_2_M; 
          } else if (step == 4) {
            state.lab_state &= ~LS_CLEANING_3_M;
            state.lab_state |= LS_CLEANING_4_M;
          }
          record_state();
          log_msg(state.last_blue_time, "clean_stage_2");
          digitalWrite(SOL_1, LOW);
          digitalWrite(SOL_3, LOW);
          
          // run p1
          digitalWrite(PUMP_POWER, LOW);
          digitalWrite(PUMP_1, HIGH);
          loop_time = millis();
          stage_started = true;
        }
        
        if (millis() - loop_time >= STAGE_2_LENGTH) {
          loop_time = 0;
          stage_started = false;
          step++;
          stage = 1;
          
          // turn everything off
          digitalWrite(PUMP_POWER, HIGH);
          digitalWrite(PUMP_1, LOW);
          
          digitalWrite(SOL_1, HIGH);
          digitalWrite(SOL_3, HIGH);
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
      if (!(state.lab_state & LS_IDLING_M)) {
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

void read_serial_input() {
  // according to NRFF Toolbox, need to wait
  // 17 ms for frame to finish transmission
  delay(17);
  int fields_read = 0;
  char* fields[NUM_FIELDS];

  // process the frame
  int i;
  for (i = 0; i < NUM_FIELDS; i++) {
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
    if (bytes != 0) {
      fields_read++;
    }
    buf[bytes] = '\0';
    log_msg(13.37, buf);
    Serial.println(i);
    fields[i] = buf;
  }
  if (fields_read != NUM_FIELDS) {
    // are we debugging?
    #ifndef DEBUG
    // spin wait until we get a new frame
    while (!Serial.available()) {} // about 50 ms
    #endif    // DEBUG
    // we might be debugging, but log either way
    log_msg(state.last_blue_time, "!serial read");
  }
  // extract what we care about
  state.blue_state = fields[0][0];
  state.last_blue_time = atof(fields[1]);
  // TODO: expand this list?
}

void read_sensors(EnvDataPtr env_data_ptr) {
  env_data_ptr->curr_data = (float) analogRead(CURR_ANALOG_PIN) * (V_REF / 1023.0) / CURR_GAIN_CONSTANT;
  // TODO: Establish a better reference voltage than "5.0" (this is currently an assumption)
  env_data_ptr->volt_data = V_REF - (float) analogRead(VOLT_ANALOG_PIN) * (V_REF / 1023.0);
  env_data_ptr->temp_data = (float) analogRead(TEMP_ANALOG_PIN) * (V_REF / 1023.0) * 100;
}

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

void log_msg(const float t, const char* msg) {
  LOG_MSG(t);
  LOG_MSG(DELIMITER);
  LOG_MSG_LN(msg);
}

uint8_t cleaning_stage(uint16_t &curr_clean_stage,
                        uint16_t &next_clean_stage,
                        uint8_t step) {
//  state.stage |= curr_clean_stage;
//  if (!(state.stage & curr_clean_stage)) {
//    // time to move to the 
//    if (step % 2 == 0) {
//      // do cleaning step 2 or 4
//      state.stage &= 
//      
//    } else {
//      // do cleaning step 1, 3, or 5
//      if (state.lab_state & LS_CLEANING_3_M) {
//          if (step == 3) {
//            state.lab_state |= LS_CLEANING_3_M;
//            state.lab_state &= ~LS_CLEANING_2_M;
//          } else if (step == 5) {
//            state.lab_state |= LS_CLEANING_5_M;
//            state.lab_state &= ~LS_CLEANING_4_M;
//          }
//          record_state();
//          log_msg(state.last_blue_time, "clean_stage_1");
//          digitalWrite(SOL_2, LOW);
//          digitalWrite(SOL_3, LOW);
//                         
//          // run p2
//          digitalWrite(PUMP_POWER, LOW);
//          digitalWrite(PUMP_2, HIGH);
//          loop_time = millis();
//          stage_started = true;
//        }
//        
//        if (millis() - loop_time >= STAGE_1_LENGTH) {
//          loop_time = 0L;
//          
//          // turn everything off
//          digitalWrite(PUMP_POWER, HIGH);
//          digitalWrite(PUMP_2, LOW);
////          LOG_MSG_LN("disabling valves 2 and 3");
//          digitalWrite(SOL_2, HIGH);
//          digitalWrite(SOL_3, HIGH);
//          if (step == 5) {
//            // update and record state
//            state.lab_state &= ~LS_CLEANING_5_M;
//            state.lab_state |= LS_CLEANED_M | LS_IDLING_M;
////            state.lab_state = LS_IDLE;
////            state.last_state = LS_CLEAN_UP;
//            record_state();
//            
//            log_msg(state.last_blue_time, "!cleaning");
//            
//            log_msg(state.last_blue_time, "clean->idle");
//  
//            // close streams
//            log_file.close();
//            SD.remove(STATE_FILE_PATH);
//          } else {
//            step++;
//          }
//        }
//    }
//  }
}
