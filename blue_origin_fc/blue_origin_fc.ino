// Copyright Eli Reed (sailedeer), released under GPLv3
// November 18th, 2019
// 
// Flight controller for MiniMEE mission. Maintains state of
// payload, facilitates communication with New Shepard,
// and controls experimentation. 

// Bonk Rewrite, September 20, 2020

#include <SPI.h>
#include "SdFat.h"
#include <BonkFramework.h>

// pin configuration macros
// TODO: going to need to update these defines
// will depend on what the chip select will end up being
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

// TODO: change this for the actual sensor we're using
#define TMP411_ADDRESS 0b1001101

// how long (in ms) it takes to prime the experiment
#define PRIME_TIME 3000

// how long (in ms) to wait before priming
#define PRIME_WAIT_TIME 1000

// how long (in ms) it takes to finish stage 1
#define STAGE_1_LENGTH 1000

// how long (in ms) it takes to finish stage 2
#define STAGE_2_LENGTH 1000

// how long to wait before taking a measurement (ms)
#define LOG_TIME_CUTOFF 250

// typedefs

// encapsulates environment data
typedef struct env_data_st {
  float volt_data;
  float curr_data;
  uint16_t temp_data;
} EnvData;

// encapsulates state information
typedef struct __attribute__((packed)) state_st {
  char blue_state;
  bool primed;
  bool priming;
  bool pump_power;
  bool pump_1;
  bool pump_2;
  bool solenoid_1;
  bool solenoid_2;
  bool solenoid_3;
  bool motor;
  bool experiment;
  bool clean;
  uint8_t clean_step;
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

SdFat sd;   // SD File System

LabState state{ 0 };

// class definitions

class MiniEventHandler : public Bonk::EventHandler {
  public:
    MiniEventHandler() : Bonk::EventHandler() {}
  protected:
    void onSeparationCommanded() const {
      // have we already primed?
      if (!state.primed) {
        // wait until things have settled down
        if (!_sep_cmd_wait()) {
          // sorry Stu, cse 143 be damned
          return;
        }
        // 
        static unsigned long prime_start_time;
        if (!state.priming) {
          // start priming
          state.priming = true;

          // TODO: start motor
          state.motor = true;
          prime_start_time = millis();
          sm.set_state(state);
        } else if (millis() - prime_start_time >= PRIME_TIME) {
          // stop priming
          state.priming = false;
          // TODO: stop motor
          state.motor = false;
          state.primed = true;
          sm.set_state(state);
        }
      }
    }

    void onCoastStart() const {
      // if primed, start experiment
      // take a measurement (every .25 seconds)
      if (state.primed) {
        static unsigned long logging_time;
        if (!state.experiment) {
          state.experiment = true;
          // TODO: start experiment          
          // TODO: initialize data logging if necessary
  
          logging_time = millis();
          sm.set_state(state);
        } else if (millis() - logging_time >= LOG_TIME_CUTOFF) {
          // take a measurement
          EnvData env_data;
          _read_sensors(env_data);
          // TODO: log data
          logging_time = millis();
        }
      } else {
        // TODO: modularize priming
      }
    }

    void onApogee() const {
      onCoastStart();
    }

    void onCoastEnd() const {
      if (state.experiment) {
        // TODO: stop the experiment
        state.experiment = false;
        sm.set_state(state);
      }
    }

    void onSafing() const {
      static unsigned long cleaning_time;
      if (!state.clean) {
        if (state.clean_step < 1) {
          // start cleaning
          _start_clean_1();
        } else {
          // figure out which step we need to start
          if (state.clean_step % 2 == 1 && millis() - cleaning_time >= STAGE_1_LENGTH) {
            if (state.clean_step == 5) {
              state.clean = true;
              sm.set_state(state);
              return;
            }
            _stop_clean_1();
            _start_clean_2();
          } else if (millis() - cleaning_time >= STAGE_2_LENGTH) {
            _stop_clean_2();
            _start_clean_1();
          }
        }
        state.clean_step++;
        sm.set_state(state);
      }

      // if (!check_lab_state(LS_CLEANED_M)) {
      //   static unsigned long clean_time;
      //   if (check_lab_state(LS_CLEANING_1_M)) {
      //     // transition to 2
      //     if (millis() - clean_time >= STAGE_1_LENGTH) {
      //       state.lab_state &= ~LS_CLEANING_1_M;
      //       state.lab_state |= LS_CLEANING_2_M;
      //       stop_cleaning(1);
      //       start_cleaning(2);
      //       clean_time = millis();
      //     }
      //     // cleaning_step_1();
      //   } else if (check_lab_state(LS_CLEANING_2_M)) {
      //     // transition to 3
      //     if (millis() - clean_time >= STAGE_2_LENGTH) {
      //       state.lab_state &= ~LS_CLEANING_2_M;
      //       state.lab_state |= LS_CLEANING_3_M;
      //       stop_cleaning(2);
      //       start_cleaning(1);
      //       clean_time = millis();
      //     }
      //   } else if (check_lab_state(LS_CLEANING_3_M)) {
      //     // transition to 4
      //     if (millis() - clean_time >= STAGE_1_LENGTH) {
      //       state.lab_state &= ~LS_CLEANING_3_M;
      //       state.lab_state |= LS_CLEANING_4_M;
      //       stop_cleaning(1);
      //       start_cleaning(2);
      //       clean_time = millis();
      //     }
      //   } else if (check_lab_state(LS_CLEANING_4_M)) {
      //     // transition to 5
      //     if (millis() - clean_time >= STAGE_2_LENGTH) {
      //       state.lab_state &= ~LS_CLEANING_4_M;
      //       state.lab_state |= LS_CLEANING_5_M;
      //       stop_cleaning(2);
      //       start_cleaning(1);
      //       clean_time = millis();
      //     }
      //   } else if (check_lab_state(LS_CLEANING_5_M)) {
      //     // end
      //     if (millis() - clean_time >= STAGE_1_LENGTH) {
      //       state.lab_state &= ~LS_CLEANING_5_M;
      //       state.lab_state |= LS_CLEANED_M | LS_IDLING_M;
      //       stop_cleaning(1);
      //       record_state();

      //       // clean up whole lab
      //       log_msg(state.last_blue_time, "!cleaning");
            
      //       log_msg(state.last_blue_time, "clean->idle");
  
      //       // close streams
      //       log_file.close();
      //       SD.remove(STATE_FILE_PATH);
      //     }
      //   } else {
      //     // we haven't done any cleaning yet
      //     state.lab_state &= ~LS_IDLING_M;
      //     state.lab_state |= LS_CLEANING_1_M;
      //     start_cleaning(1);
      //     clean_time = millis();
      //   }
      // }
    }

    void onTouchdown() const {
      onSafing();
    }

  private:
    // poll sensors for current environmental data. Stores results
    // in EnvData struct env_data
    void _read_sensors(EnvData& env_data) const {
      env_data.curr_data = m226.readShuntCurrent();
      env_data.volt_data = m226.readShuntVoltage();
      env_data.temp_data = thermometer.readLocalTemperature();
    }

    // determines if we can start priming. returns true if so, false otherwise
    bool _sep_cmd_wait() const {
      static unsigned long sep_cmd_time = millis();
      return (millis() - sep_cmd_time >= PRIME_WAIT_TIME);
    }

    void _start_clean_1() const {
      state.solenoid_2 = true;
      state.solenoid_3 = true;

      state.pump_power = true;
      state.pump_2 = true;

      // TODO: turn on pumps/solenoids
      // digitalWrite(SOL_2, LOW);
      // digitalWrite(SOL_3, LOW);
                      
      // // run p2
      // digitalWrite(PUMP_POWER, LOW);
      // digitalWrite(PUMP_2, HIGH);
    }

    void _stop_clean_1() const {
      state.solenoid_2 = false;
      state.solenoid_3 = false;

      state.pump_power = false;
      state.pump_2 = false;

      // TODO: turn off pumps/solenoids
      // digitalWrite(SOL_2, HIGH);
      // digitalWrite(SOL_3, HIGH);
                      
      // // run p2
      // digitalWrite(PUMP_POWER, HIGH);
      // digitalWrite(PUMP_2, LOW);
    }

    void _start_clean_2() const {
      state.solenoid_1 = true;
      state.solenoid_3 = true;

      state.pump_power = true;
      state.pump_1 = true;

      // TODO: turn on pumps/solenoids
      // digitalWrite(SOL_1, LOW);
      // digitalWrite(SOL_3, LOW);

      // // run p1
      // digitalWrite(PUMP_POWER, LOW);
      // digitalWrite(PUMP_1, HIGH);
    }

    void _stop_clean_2() const {
      state.solenoid_1 = false;
      state.solenoid_3 = false;

      state.pump_power = false;
      state.pump_1 = false;

      // TODO: turn off pumps/solenoids
      // digitalWrite(SOL_1, HIGH);
      // digitalWrite(SOL_3, HIGH);

      // // run p1
      // digitalWrite(PUMP_POWER, HIGH);
      // digitalWrite(PUMP_1, LOW);
    }
};

MiniEventHandler eh;
// end global variables

// initialize pin 
void pin_init() {

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
}

// configures and initializes serial, sd,
// pump, solenoid, experiment interfaces
void setup() {
  // initialize Serial connection
  Serial.begin(115200, SERIAL_8N1);
  while (!Serial);

  state.blue_state = '@';
  lm.begin("log.txt", "data.txt");
  sm.begin("state.txt", state);
  m226.begin();
  thermometer.begin();
  containmentPins.begin();
  #ifdef BONK_BOOST
  boost226.begin();
  #endif  // BONK_BOOST
  
  // configure pins
  pin_init();
}

void loop() {
  eh.tick();
}
