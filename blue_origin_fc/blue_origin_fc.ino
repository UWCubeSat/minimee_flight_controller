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
#define CHIP_SELECT 10
#define TEMP_ANALOG_PIN A0
#define CURR_ANALOG_PIN A1
#define VOLT_ANALOG_PIN A2

#define PUMP_POWER 4
#define PUMP_1 5
#define PUMP_2 6

// TODO: Decide on pin configuration for solenoids, motor
//#define SOL_1
//#define SOL_2
//#define SOL_3
//
//#define MOTOR

#define EXPERIMENT 9

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

// how long to wait before taking another measurement
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

// bit masks for stage
#define IDLING      0x0001  // 1 << 0
#define PRIMING     0x0002  // 1 << 1
#define PRIMED      0x0004  // 1 << 2
#define PLATING     0x0008  // 1 << 3
#define PLATED      0x0010  // 1 << 4
#define CLEANING_1  0x0020  // 1 << 5
#define CLEANING_2  0x0040  // 1 << 6
#define CLEANING_3  0x0080  // 1 << 7
#define CLEANING_4  0x0100  // 1 << 8
#define CLEANING_5  0x0200  // 1 << 9
#define CLEANED     0x0400  // 1 << 10


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
  uint16_t stage;
  uint8_t lab_state;
  char blue_state;
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

// stores state information
State state;

// global logging file
File log_file;

// global data file
File data_file;

// end global variables


// debug macros

// #define DEBUG

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
    state.stage = IDLING;
    state.lab_state = LS_IDLE;
    state.blue_state = '@';
    state.last_state = LS_NO_STATE;
    log_msg(state.last_blue_time, "cold");
  }
  
  // configure pins
  pin_init();
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
  pinMode(PUMP_POWER, OUTPUT);
  pinMode(PUMP_1, OUTPUT);
  pinMode(PUMP_2, OUTPUT);
  pinMode(EXPERIMENT, OUTPUT);

  // pin for controlling motor
//  pinMode(MOTOR, OUTPUT);

  // pins for solenoids
//  pinMode(SOL_1, OUTPUT);
//  pinMode(SOL_2, OUTPUT);
//  pinMode(SOL_3, OUTPUT):

  // pins for sensor reading
  pinMode(TEMP_ANALOG_PIN, INPUT);
  pinMode(CURR_ANALOG_PIN, INPUT);
  pinMode(VOLT_ANALOG_PIN, INPUT);
  digitalWrite(EXPERIMENT, HIGH);

  // valves are active low
  // pumps are active high
  // experiment is active low
  // TODO: active state of motor?

  log_msg(state.last_blue_time, "pins");
}

// main state machine logic
void loop() {
  if (Serial.available() > 0) {
    read_serial_input();
  }
  
  switch(state.lab_state) {
    
    case LS_IDLE:
      {     
        // start priming?
        if (state.blue_state == BS_SEP_COMMANDED) {
          state.lab_state = LS_PRIME_EXPERIMENT;
          state.last_state = LS_IDLE;
          state.stage |= PRIMING;
          state.stage &= !IDLING;
          record_state();
          
          log_msg(state.last_blue_time, "idle->priming");
        }

        // can we start the experiment?
        else if (state.blue_state == BS_COAST_START ||
                    state.blue_state == BS_APOGEE) {
          state.lab_state = LS_CELL_PLATING;
          state.last_state = LS_IDLE;
          state.stage |= PLATING;
          state.stage &= !IDLING;
          record_state();
          
          log_msg(state.last_blue_time, "idle->plating");
        }

        // have we landed?
        else if (state.blue_state == BS_LANDING ||
                state.blue_state == BS_SAFING) {
          state.lab_state = LS_CLEAN_UP;
          state.last_state = LS_IDLE;
          state.stage |= CLEANING_1;
          state.stage &= !IDLING;
          record_state();
          
          log_msg(state.last_blue_time, "idle->clean");
        }
      }
    break;
    
    case LS_PRIME_EXPERIMENT:
      {
        static long prime_commanded_time = millis();
        static long prime_start_time = 0L;
        
        // prime time?
        if (millis() - prime_commanded_time >= PRIME_WAIT_TIME) {
          prime_commanded_time = 0L;
          log_msg(state.last_blue_time, "priming");

          // engage the motor
//          digitalWrite(MOTOR, HIGH);
          prime_start_time = millis();
        }
        // done priming?
        else if (millis() - prime_start_time >= PRIME_TIME) {
          state.stage |= PRIMED;
          state.stage &= !PLATING;
          state.stage |= IDLING;
          state.lab_state = LS_IDLE;
          state.last_state = LS_PRIME_EXPERIMENT;
          record_state();   
          log_msg(state.last_blue_time, "!priming");

          // turn off the motor
          //digitalWrite(MOTOR, LOW);
          
          log_msg(state.last_blue_time, "priming->idle");
        }
      }
    break;
      
    case LS_CELL_PLATING:
      {
        static long last_log_time = 0L;
        static bool started = false;
        
        if (!started) {
          log_msg(state.last_blue_time, "plating");

          // initialize experiment
          digitalWrite(EXPERIMENT, LOW);
          started = true;
          
          data_file = SD.open(DATA_FILE_PATH, FILE_WRITE);
          last_log_time = millis();
        }
        // time to measure?
        else if (millis() - last_log_time >= LOG_TIME_CUTOFF) {
          last_log_time = millis();
          EnvData env_data;
          read_sensors(&env_data);
          log_sensor_data(&env_data, data_file);
        }
        // time to stop plating?
        else if (state.blue_state == BS_COAST_END) {
          state.lab_state = LS_IDLE;
          state.last_state = LS_CELL_PLATING;
          state.stage |= IDLING;
          state.stage |= PLATED;
          state.stage &= !PLATING;
          record_state();
          
          log_msg(state.last_blue_time, "!plating");
          log_msg(state.last_blue_time, "plating->idle");
          
          // clean up
          data_file.close();
          digitalWrite(EXPERIMENT, HIGH);
        }
      }
      break;

    case LS_CLEAN_UP:
      {
        static uint8_t stage = 1;
        static uint8_t step = 1;
        static bool stage_started = false;
        static long pump_start_time = 0L;

        if (stage == 1) {
          if (!stage_started) {
            if (step == 3) {
              state.stage |= CLEANING_3;
              state.stage &= !CLEANING_2;
              record_state();
            } else if (step == 5) {
              state.stage |= CLEANING_5;
              state.stage &= !CLEANING_4;
            }
            log_msg(state.last_blue_time, "clean_stage_1");
            // digitalWrite(SOL_2, LOW);
            // digitalWrite(SOL_3, LOW);
                           
            // run p2
            digitalWrite(PUMP_POWER, HIGH);
            digitalWrite(PUMP_2, HIGH);
            pump_start_time = millis();
          }
          
          if (millis() - pump_start_time >= STAGE_1_LENGTH) {
            pump_start_time = 0L;
            stage_started = false;
            step++;
            
            // turn everything off
            digitalWrite(PUMP_POWER, HIGH);
            digitalWrite(PUMP_2, HIGH);

            // digitalWrite(SOL_2, HIGH);
            // digitalWrite(SOL_3, HIGH);
            if (step == 5) {
              // update and record state
              state.cleaned = true;
              state.lab_state = LS_IDLE;
              state.last_state = LS_CLEAN_UP;
              record_state();
              log_msg(state.last_blue_time, "!cleaning");
              
              digitalWrite(PUMP_POWER, LOW);
              digitalWrite(PUMP_2, LOW);
              log_msg(state.last_blue_time, "clean->idle");

              // close streams
              log_file.close();
              SD.remove(STATE_FILE_PATH);
            } else {
              // advance to stage 2
              stage++;
            }
          }
        } else if (stage == 2) {
          if (!stage_started) {
            if (step == 2) {
              
            }
            log_msg("clean_stage_2");
            // TODO: open S1 & S3
            // digitalWrite(SOL_1, LOW);
            // digitalWrite(SOL_3, LOW);
            
            // run p1
            digitalWrite(PUMP_POWER, HIGH);
            digitalWrite(PUMP_1, HIGH);
            pump_start_time = millis();
          }
          
          if (millis() - pump_start_time >= STAGE_2_LENGTH) {
            pump_start_time = 0;
            stage_started = false;
            step++;
            stage--;
            
            // turn everything off
            digitalWrite(PUMP_POWER, HIGH);
            digitalWrite(PUMP_1, HIGH);

            // digitalWrite(SOL_1, LOW);
            // digitalWrite(SOL_3, LOW);
          }
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
        state.lab_state = LS_IDLE;
        state.last_state = LS_NO_STATE;
        state.stage |= IDLING;
        record_state();
        log_msg(state.last_blue_time, "null->idle");
      }
    break;
  }
}

void restore_state() {
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
}

void record_state() {
  if (SD.exists(STATE_FILE_PATH)) {
    SD.remove(STATE_FILE_PATH);
  }
  File state_file = SD.open(STATE_FILE_PATH, FILE_WRITE);
  state_file.print(state.lab_state);
  state_file.print(DELIMITER);
  state_file.print(state.last_state);
  state_file.print(DELIMITER);
  state_file.print(state.blue_state);
  state_file.println(DELIMITER);
  state_file.close();
}

void read_serial_input() {
  // according to NRFF Toolbox, need to wait
  // 17 ms for frame to finish transmission
  delay(17);
  int bytes_read = 0;
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
    buf[bytes] = '\0';
    fields[i] = buf;
  }
  // extract what we care about
  state.blue_state = fields[0][0];
  state.last_blue_time = atof(fields[1]);
}

void read_sensors(EnvDataPtr env_data_ptr) {
  env_data_ptr->curr_data = (float) analogRead(CURR_ANALOG_PIN) * (5.0 / 1023.0) / CURR_GAIN_CONSTANT;
  // TODO: Establish a better reference voltage than "5.0" (this is currently an assumption)
  env_data_ptr->volt_data = 5.0 - (float) analogRead(VOLT_ANALOG_PIN) * (5.0 / 1023.0);
  env_data_ptr->temp_data = (float) analogRead(TEMP_ANALOG_PIN) * (5.0 / 1023.0) * 100;
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
