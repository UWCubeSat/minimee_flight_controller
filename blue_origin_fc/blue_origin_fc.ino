// Copyright Eli Reed (sailedeer), released under GPLv3
// November 18th, 2019
//
// Flight controller for MiniMEE mission. Maintains state of
// payload, facilitates communication with New Shepard,
// and controls experimentation.

// Bonk Rewrite, September 20, 2020

#include <SPI.h>
#include <Wire.h>
#include "SdFat.h"
#include <BonkFramework.h>

// pin configuration macros
// TODO: going to need to update these defines
// will depend on what the chip select will end up being
// and what containment pins
#define CHIP_SELECT 4

#define PUMP_POWER 0
#define PUMP_1 1
#define PUMP_2 2

#define SOL_1 3
#define SOL_2 4
#define SOL_3 5
#define SOL_4 6

#define FLUID_SENSOR_1 7
#define FLUID_SENSOR_2 8

#define EXPERIMENT 9

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

// how many times do we clean
#define RINSES 2

enum fluid { H20, CuSO4 };

// typedefs

// encapsulates environment data
typedef struct env_data_st {
  float volt_data;
  float curr_data;
  uint16_t local_temp_data;
  uint16_t remote_temp_data;
} EnvData;

// encapsulates state information
typedef struct __attribute__((packed)) state_st {
  bool primed;
  bool pump_powered;
  bool pump_1_on;
  bool pump_2_on;
  bool solenoid_1_open;
  bool solenoid_2_open;
  bool solenoid_3_open;
  bool solenoid_4_open;
  bool cell_full;
  bool experimenting;
  bool clean;
  uint8_t rinses;
} LabState;

// end typedefs

#ifdef DEBUG_MODE
#define NOTIFY DEBUG
#define WARNING DEBUG
#define ERROR DEBUG
#endif  // DEBUG_MODE

// global variables

Bonk::StateManager<LabState> sm;
Bonk::LogManager lm;
Bonk::Main226 m226;
Bonk::Tmp411 thermometer(TMP411_ADDRESS);
Bonk::Pca9557 containmentPins(BONK_CONTAINMENT9557_ADDRESS);
Bonk::Boost226 boost226;

SdFat sd;   // SD File System

LabState state{ 0 };

// class definitions

class MiniEventHandler : public Bonk::EventHandler {
  public:
    MiniEventHandler() : Bonk::EventHandler() {}
  protected:
    void onLiftoff() const {
      if (!state.primed) {
        _fill_cell(CuSO4);
      }
    }

    void onCoastStart() const {
      // if primed, start experiment
      // take a measurement (every .25 seconds)
      if (state.primed) {
        static unsigned long logging_time;
        if (!state.experimenting) {
          containmentPins.digitalWrite(EXPERIMENT, HIGH);
          state.experimenting = true;
          logging_time = millis();
          sm.set_state(state);
          lm.log(Bonk::LogType::NOTIFY, "plating");
        } else if (millis() - logging_time >= LOG_TIME_CUTOFF) {
          // take a measurement
          EnvData env_data;
          _read_sensors(env_data);
          _log_data(env_data);
          logging_time = millis();
        }
      }
    }

    void onApogee() const {
      onCoastStart();
    }

    void onCoastEnd() const {
      if (state.experimenting) {
        containmentPins.digitalWrite(EXPERIMENT, HIGH);
        state.experimenting = false;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, "finished plating");
      } else if (!state.clean) {
        if (state.rinses < RINSES) {
          if (state.cell_full) {
            _drain_cell((state.rinses == 0) ? CuSO4 : H20);
          } else {
            _fill_cell(H20);
          }
        }
      }
    }

  private:
    // poll sensors for current environmental data. Stores results
    // in EnvData struct env_data
    void _read_sensors(EnvData& env_data) const {
      // TODO: we aren't using shunt current or shunt voltage to take
      // experimental measurements, update these to the real thing
      env_data.curr_data = m226.readShuntCurrent();
      env_data.volt_data = m226.readShuntVoltage();
      env_data.local_temp_data = thermometer.readLocalTemperature();
      env_data.remote_temp_data = thermometer.readRemoteTemperature();
    }

    void _log_data(EnvData& env_data) const {
      // TODO: implement this function
    }
    
    void _fill_cell(enum fluid fluidType) const {
      uint8_t solenoid_pin;
      bool *solenoid_state_ptr;

      if (fluidType == H20) {
        solenoid_pin = SOL_2;
        solenoid_state_ptr = &state.solenoid_2_open;
      } else if (fluidType == CuSO4) {
        solenoid_pin = SOL_1;
        solenoid_state_ptr = &state.solenoid_1_open;
      } else {
        lm.log(Bonk::LogType::ERROR, "that's no solenoid!");
        return;
      }

      if (!(*solenoid_state_ptr)) {
        containmentPins.digitalWrite(solenoid_pin, LOW);
        *solenoid_state_ptr = true;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, (solenoid_pin == SOL_1) ? "valve 1 open" : "valve 2 open");
      } else if (!state.pump_1_on) {
        containmentPins.digitalWrite(PUMP_POWER, LOW);
        containmentPins.digitalWrite(PUMP_1, HIGH);
        state.pump_1_on = true;
        state.pump_powered = true;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, "pump 1 on");
      } else if (!containmentPins.digitalRead(FLUID_SENSOR_1)) {
        containmentPins.digitalWrite(PUMP_POWER, HIGH);
        containmentPins.digitalWrite(PUMP_1, LOW);
        containmentPins.digitalWrite(solenoid_pin, HIGH);
        state.pump_powered = false;
        state.pump_1_on = false;
        *solenoid_state_ptr = false;
        state.cell_full = true;
        state.primed = true;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, "filled");
      }
    }

    void _drain_cell(enum fluid fluidType) const {
      uint8_t solenoid_pin;
      bool *solenoid_state_ptr;

      if (fluidType == H20) {
        solenoid_pin = SOL_3;
        solenoid_state_ptr = &state.solenoid_3_open;
      } else if (fluidType == CuSO4) {
        solenoid_pin = SOL_4;
        solenoid_state_ptr = &state.solenoid_4_open;
      } else {
        lm.log(Bonk::LogType::ERROR, "that's no solenoid!");
        return;
      }

      if (!(*solenoid_state_ptr)) {
        containmentPins.digitalWrite(solenoid_pin, LOW);
        *solenoid_state_ptr = true;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, (solenoid_pin == SOL_3) ? "valve 3 open" : "valve 4 open");
      } else if (!state.pump_2_on) {
        containmentPins.digitalWrite(PUMP_POWER, LOW);
        containmentPins.digitalWrite(PUMP_2, HIGH);
        state.pump_2_on = true;
        state.pump_powered = true;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, "pump 2 on");
      } else if (!containmentPins.digitalRead(FLUID_SENSOR_2)) {
        containmentPins.digitalWrite(PUMP_POWER, HIGH);
        containmentPins.digitalWrite(PUMP_2, LOW);
        containmentPins.digitalWrite(solenoid_pin, HIGH);
        state.pump_powered = false;
        state.pump_1_on = false;
        *solenoid_state_ptr = false;
        state.cell_full = false;
        state.primed = false;
        sm.set_state(state);
        lm.log(Bonk::LogType::NOTIFY, "drained");
      }
    }
};

MiniEventHandler eh;
// end global variables

// initialize pin
void pin_init() {

  // initialize pump pins
  containmentPins.pinMode(PUMP_POWER, OUTPUT);
  containmentPins.pinMode(PUMP_1, OUTPUT);
  containmentPins.pinMode(PUMP_2, OUTPUT);

  // initialize experiment pin
  containmentPins.pinMode(EXPERIMENT, OUTPUT);

  // initialize solenoid pins
  containmentPins.pinMode(SOL_1, OUTPUT);
  containmentPins.pinMode(SOL_2, OUTPUT);
  containmentPins.pinMode(SOL_3, OUTPUT);
  containmentPins.pinMode(SOL_4, OUTPUT);

  // initialize fluid sensor pins
  containmentPins.pinMode(FLUID_SENSOR_1, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_2, INPUT);

  // valves are active low
  // pumps are active high
  // pump power active low
  // experiment is active low
  containmentPins.digitalWrite(PUMP_POWER, HIGH);
  containmentPins.digitalWrite(PUMP_1, LOW);
  containmentPins.digitalWrite(PUMP_2, LOW);
  containmentPins.digitalWrite(SOL_1, HIGH);
  containmentPins.digitalWrite(SOL_2, HIGH);
  containmentPins.digitalWrite(SOL_3, HIGH);
  containmentPins.digitalWrite(SOL_4, HIGH);
  containmentPins.digitalWrite(EXPERIMENT, HIGH);

  lm.log(Bonk::LogType::NOTIFY, "pins initialized");
}

// configures and initializes serial, sd,
// pump, solenoid, experiment interfaces
void setup() {
  // initialize Wire for containment pins
  Wire.begin();
  Wire.setClock(400000);

  // initialize Serial connection with New Shepard
  Serial.begin(115200, SERIAL_8N1);

  // initialize SdFat
  sd.begin(CHIP_SELECT);

  lm.begin("log.txt", "data.txt");
  sm.begin("state.txt", state);
  // TDOD: update shunt resistor, current limit values
  m226.begin();
  thermometer.begin();
  containmentPins.begin();
  Bonk::enableBoostConverter(true);
  boost226.begin();

  // configure pins
  lm.log(Bonk::LogType::NOTIFY, "BONK initialized");
  pin_init();
}

void loop() {
  eh.tick();
}
