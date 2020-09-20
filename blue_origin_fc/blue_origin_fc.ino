// Copyright Eli Reed (sailedeer), released under GPLv3
// November 18th, 2019
// 
// Flight controller for MiniMEE mission. Maintains state of
// payload, facilitates communication with New Shepard,
// and controls experimentation. 

// Bonk Rewrite, September 20, 2020

#include <BonkFramework.h>

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

#define TMP411_ADDRESS 0b1001101

// how long (in ms) it takes to prime the experiment
#define PRIME_TIME 3000

// how long (in ms) to wait before priming
#define PRIME_WAIT_TIME 1000

// how long (in ms) it takes to finish stage 1
#define STAGE_1_LENGTH 1000

// how long (in ms) it takes to finish stage 2
#define STAGE_2_LENGTH 1000

// file names for logging, keeping track of state, etc.
#define LOG_FILE_PATH "log.txt"
#define STATE_FILE_PATH "state.txt"
#define DATA_FILE_PATH "data.txt"

// typedefs

// encapsulates environment data
typedef struct env_data_st {
  float volt_data;
  float curr_data;
  float temp_data;
} EnvData;

// encapsulates state information
typedef struct __attribute__((packed)) state_st {
  char blue_state;
  bool pump_power;
  bool pump_1;
  bool pump_2;
  bool solenoid_1;
  bool solenoid_2;
  bool solenoid_3;
  bool motor;
  bool experiment;
} LabState;

// end typedefs


// global variables

Bonk::StateManager<LabState> sm;
Bonk::LogManager lm;
Bonk::Main226 m226;
Bonk::Tmp411 thermometer(TMP411_ADDRESS);
Bonk::Pca9557 containmentPins(BONK_CONTAINMENT9557_ADDRESS);
#ifdef BONK_BOOST
Bonk::Boost226 boost226;
#endif  // BONK_BOOST;

// end global variables

void init() {
    LabState def_state{0};
    def_state.blue_state = '@';
    sm.begin(LOG_FILE_PATH, DATA_FILE_PATH);
    lm.begin(STATE_FILE_PATH, def_state);
    m226.begin();
    thermometer.begin();
    containmentPins.begin();
    #ifdef BONK_BOOST
    boost226.begin();
    #endif  // BONK_BOOST
}

// // initialize pin 
// void pin_init() {
//   // nominally, this is 1.1V (and won't change much)
//   analogReference(INTERNAL);

//   // initialize pump pins
//   pinMode(PUMP_POWER, OUTPUT);
//   pinMode(PUMP_1, OUTPUT);
//   pinMode(PUMP_2, OUTPUT);

//   // initialize experiment pin
//   pinMode(EXPERIMENT, OUTPUT);

//   // initialize motor pin
//   pinMode(MOTOR, OUTPUT);

//   // initialize solenoid pins
//   pinMode(SOL_1, OUTPUT);
//   pinMode(SOL_2, OUTPUT);
//   pinMode(SOL_3, OUTPUT);

//   // pins for sensor reading
//   pinMode(TEMP_ANALOG_PIN, INPUT);
//   pinMode(CURR_ANALOG_PIN, INPUT);
//   pinMode(VOLT_ANALOG_PIN, INPUT);

//   // valves are active low
//   // pumps are active high
//   // pump power active low
//   // motor active low
//   // experiment is active low
//   // TODO: active state of motor?
//   digitalWrite(PUMP_POWER, HIGH);
//   digitalWrite(PUMP_1, LOW);
//   digitalWrite(PUMP_2, LOW);
//   digitalWrite(SOL_1, HIGH);
//   digitalWrite(SOL_2, HIGH);
//   digitalWrite(SOL_3, HIGH);
//   digitalWrite(MOTOR, HIGH);
//   digitalWrite(EXPERIMENT, HIGH);

//   log_msg(state.last_blue_time, "pins");
// }

// poll sensors for current environmental data. Stores results
// in EnvData struct env_data
// void read_sensors(EnvData &env_data) {
//   env_data.curr_data = static_cast<float>(analogRead(CURR_ANALOG_PIN) * (V_REF / 1023.0) / CURR_GAIN_CONSTANT);
//   env_data.volt_data = V_REF - static_cast<float>(analogRead(VOLT_ANALOG_PIN) * (V_REF / 1023.0));
//   env_data.temp_data = static_cast<float>(analogRead(TEMP_ANALOG_PIN) * (V_REF / 1023.0) * 100);
// }

// // write polled sensor data to file
// void log_sensor_data(const EnvData &env_data, File data_file) {
//   // convert sensor values to strings
//   char s_volt[10];
//   char s_curr[10];
//   char s_temp[10];
//   dtostrf(env_data.volt_data, 5, 3, s_volt);
//   dtostrf(env_data.curr_data, 5, 3, s_curr);
//   dtostrf(env_data.temp_data, 5, 3, s_temp);

//   // write sensor data to data file
//   #ifdef DEBUG
//     LOG_MSG(state.last_blue_time);
//     LOG_MSG(DELIMITER);
//     LOG_MSG(s_volt);
//     LOG_MSG(DELIMITER);
//     LOG_MSG(s_curr);
//     LOG_MSG(DELIMITER);
//     LOG_MSG_LN(s_temp);
//   #else
//     data_file.print(state.last_blue_time);
//     data_file.print(DELIMITER);
//     data_file.print(s_volt);
//     data_file.print(DELIMITER);
//     data_file.print(s_curr);
//     data_file.print(DELIMITER);
//     data_file.println(s_temp);
//   #endif    // DEBUG
// }

// starts cleaning step specified by i
void start_cleaning(uint8_t i) {
  if (i == 1) {
    digitalWrite(SOL_2, LOW);
    digitalWrite(SOL_3, LOW);
                   
    // run p2
    digitalWrite(PUMP_POWER, LOW);
    digitalWrite(PUMP_2, HIGH);
    log_msg(state.last_blue_time, "clean1");
  } else if (i == 2) {
    digitalWrite(SOL_1, LOW);
    digitalWrite(SOL_3, LOW);
  
    // run p1
    digitalWrite(PUMP_POWER, LOW);
    digitalWrite(PUMP_1, HIGH);
    log_msg(state.last_blue_time, "clean2");
  } else {
    log_msg(state.last_blue_time, "invalid cleaning step");
  }
}

// stops cleaning step specified by i
void stop_cleaning(uint8_t i) {
  if (i == 1) {
    digitalWrite(SOL_2, HIGH);
    digitalWrite(SOL_3, HIGH);
                   
    // run p2
    digitalWrite(PUMP_POWER, HIGH);
    digitalWrite(PUMP_2, LOW);
    log_msg(state.last_blue_time, "!clean1");
  } else if (i == 2) {
    digitalWrite(SOL_1, HIGH);
    digitalWrite(SOL_3, HIGH);
  
    // run p1
    digitalWrite(PUMP_POWER, HIGH);
    digitalWrite(PUMP_1, LOW);
    log_msg(state.last_blue_time, "!clean2");
  } else {
    log_msg(state.last_blue_time, "invalid cleaning step");
  }
}

// determines if we can start priming. returns true if so, false otherwise
bool sep_cmd_wait() {
  static unsigned long sep_cmd_time = millis();
  return (millis() - sep_cmd_time >= PRIME_WAIT_TIME);
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

  // check to see if blue's state updated when we read
  if (state.blue_state != state.last_blue_state) {
    record_state();
  }

  switch(state.blue_state) {

    case BS_SEP_COMMANDED:    // CC is free
    {
      // have we already primed?
      if (!check_lab_state(LS_PRIMED_M)) {
        
        // wait until things have settled down
        if (!sep_cmd_wait())
          break;
          
        // 
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
          read_sensors(env_data);
          log_sensor_data(env_data, data_file);
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
            stop_cleaning(1);
            start_cleaning(2);
            clean_time = millis();
          }
          // cleaning_step_1();
        } else if (check_lab_state(LS_CLEANING_2_M)) {
          // transition to 3
          if (millis() - clean_time >= STAGE_2_LENGTH) {
            state.lab_state &= ~LS_CLEANING_2_M;
            state.lab_state |= LS_CLEANING_3_M;
            stop_cleaning(2);
            start_cleaning(1);
            clean_time = millis();
          }
        } else if (check_lab_state(LS_CLEANING_3_M)) {
          // transition to 4
          if (millis() - clean_time >= STAGE_1_LENGTH) {
            state.lab_state &= ~LS_CLEANING_3_M;
            state.lab_state |= LS_CLEANING_4_M;
            stop_cleaning(1);
            start_cleaning(2);
            clean_time = millis();
          }
        } else if (check_lab_state(LS_CLEANING_4_M)) {
          // transition to 5
          if (millis() - clean_time >= STAGE_2_LENGTH) {
            state.lab_state &= ~LS_CLEANING_4_M;
            state.lab_state |= LS_CLEANING_5_M;
            stop_cleaning(2);
            start_cleaning(1);
            clean_time = millis();
          }
        } else if (check_lab_state(LS_CLEANING_5_M)) {
          // end
          if (millis() - clean_time >= STAGE_1_LENGTH) {
            state.lab_state &= ~LS_CLEANING_5_M;
            state.lab_state |= LS_CLEANED_M | LS_IDLING_M;
            stop_cleaning(1);
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
          start_cleaning(1);
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
