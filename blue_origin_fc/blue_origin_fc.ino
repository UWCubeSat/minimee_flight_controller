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

#define PUMP_POWER A3
#define PUMP_TOGGLE 4

#define PUMP_1 LOW
#define PUMP_2 HIGH
#define PUMP_POWER_ON LOW
#define PUMP_POWER_OFF HIGH

#define SOL_1 5
#define SOL_2 6
// TODO: move this back to pin 7
#define SOL_3 7

// TODO: flip these for the real thing
#define SOL_ON LOW
#define SOL_OFF HIGH

#define FLUID_SENSOR_1_ON 0  // when first sensor is definitely on
#define FLUID_SENSOR_1_OFF 1  // when it's definitely off
#define FLUID_SENSOR_2_ON 2   // when second sensor is definitely on
#define FLUID_SENSOR_2_OFF 3  // when it's definitely off

// two experiments, but triggered by the same pin
#define EXPERIMENT A2

#define EXPERIMENT_OFF HIGH
#define EXPERIMENT_ON LOW

// TODO: change this for the actual sensor we're using
#define TMP411_ADDRESS 0b1001100

// how long to wait before taking a measurement (ms)
#define LOG_TIME_CUTOFF 250

// how long to wait before checking fluid sensor
#define DRAIN_TIME_CUTOFF 250

// how many times do we clean
#define RINSES 2

enum action {
  IDLE,
  START_SULF,
  START_WATER,
  STOP_SULF,
  STOP_WATER,
  CHECK_FULL,
  START_DRAIN,
  CHECK_DRAIN,
  STOP_DRAIN,
  DRAIN_WAIT,
  START_EXP,
  COLLECT_DATA,
  STOP_EXP,
};

// typedefs

// encapsulates environment data
typedef struct data_st {
  float voltage;
  float current;
  uint16_t local_temperature;
  uint16_t remote_temperature;
} Data;

// encapsulates state information
typedef struct __attribute__((packed)) state_st {
  bool abort;
  bool primed;
  bool pump_powered;
  bool pump_toggle;
  bool solenoid_1_open;
  bool solenoid_2_open;
  bool solenoid_3_open;
  bool cell_full;
  bool experimenting;
  bool clean;
  uint8_t rinses;
  enum action current_action;
  enum action last_action;
} LabState;

// end typedefs
#define DEBUG_MODE
#ifdef DEBUG_MODE
#define NOTIFY DEBUG
#define WARNING DEBUG
#define ERROR DEBUG
#endif  // DEBUG_MODE

// global variables

Bonk::StateManager<LabState> sm;
Bonk::LogManager lm;
Bonk::Main226 m226;
Bonk::Experiment226 chip1;
Bonk::Experiment226 chip2;
Bonk::Tmp411 thermometer(TMP411_ADDRESS);
Bonk::Pca9557 containmentPins(BONK_CONTAINMENT9557_ADDRESS);
Bonk::Boost226 boost226;

SdFat sd;   // SD File System

LabState state{ 0 };

FatFile data_file;

Bonk::EventHandler eh;
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

  // initialize fluid sensor pins
  containmentPins.pinMode(FLUID_SENSOR_1_ON, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_1_OFF, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_2_ON, INPUT);
  containmentPins.pinMode(FLUID_SENSOR_2_OFF, INPUT);

  // valves are active low
  // pump 1 is low, pump 2 is high
  // pump power active low
  // experiment is active low
  containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_OFF);
  containmentPins.digitalWrite(PUMP_TOGGLE, PUMP_1);
  containmentPins.digitalWrite(SOL_1, SOL_OFF);
  containmentPins.digitalWrite(SOL_2, SOL_OFF);
  containmentPins.digitalWrite(SOL_3, SOL_OFF);
  containmentPins.digitalWrite(EXPERIMENT, EXPERIMENT_OFF);

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

  lm.begin("log.txt");
  sm.begin("state.txt", state);
  // TDOD: update shunt resistor, current limit values
//  m226.begin();
  thermometer.begin();
  containmentPins.begin();
  Bonk::enableBoostConverter(false);
  boost226.begin();

  chip1.begin();
  chip2.begin();

  // configure pins
  lm.log(Bonk::LogType::NOTIFY, "BONK initialized");
  pin_init();

  LabState temp;
  if (sm.get_state(temp)) {
    state = temp;
  }

  // TODO: add extra logic for restoring state/turning on hardware
}

void start_sulf() {
  if (!state.solenoid_1_open && !state.pump_powered) {
    // open the CuSO4 valve and turn on pump 1
    containmentPins.digitalWrite(SOL_1, SOL_ON);
    containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_OFF);
    containmentPins.digitalWrite(PUMP_TOGGLE, PUMP_1);

    // update our state
    state.solenoid_1_open = true;
    state.pump_powered = true;
    state.pump_toggle = false;
    sm.set_state(state);

    // log that we opened the solenoid
    lm.log(Bonk::LogType::NOTIFY, "sulfur fill started");
  }
}

void stop_sulf() {
  if (state.pump_powered && state.solenoid_1_open) {
    // turn off the pump
    containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_OFF);
    containmentPins.digitalWrite(SOL_1, SOL_OFF);

    // update our state
    state.solenoid_1_open = false;
    state.pump_powered = false;
    sm.set_state(state);

    // log that we turned off the fill pump
    lm.log(Bonk::LogType::NOTIFY, "sulf fill stopped");
  }
}

bool check_full() {
  bool def_on = containmentPins.digitalRead(FLUID_SENSOR_1_ON) == HIGH;
  bool def_off = containmentPins.digitalRead(FLUID_SENSOR_1_OFF) == HIGH;
  return def_on && def_off;
}

bool check_drain() {
  bool def_on = containmentPins.digitalRead(FLUID_SENSOR_2_ON) == HIGH;
  bool def_off = containmentPins.digitalRead(FLUID_SENSOR_2_OFF) == HIGH;
  return def_on && def_off;
}

void start_water() {
  if (!state.solenoid_2_open && !state.pump_powered) {
    // open the water valve
    containmentPins.digitalWrite(SOL_2, SOL_ON);
    containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_ON);
    containmentPins.digitalWrite(PUMP_TOGGLE, PUMP_2);

    // update our state
    state.solenoid_2_open = true;
    state.pump_toggle = true;
    state.pump_powered = true;
    sm.set_state(state);

    // log that we opened the solenoid
    lm.log(Bonk::LogType::NOTIFY, "water fill started");
  } 
}

void stop_water() {
  if (state.pump_powered && state.solenoid_2_open) {
    // turn off the fill pump
    containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_OFF);
    containmentPins.digitalWrite(SOL_2, SOL_OFF);

    // update our state
    state.pump_powered = false;
    state.solenoid_2_open = false;
    sm.set_state(state);

    // log that we turned off the fill pump
    lm.log(Bonk::LogType::NOTIFY, "water fill stopped");
  }
}

void start_exp() {
  if (!state.experimenting) {
    // turn on the experiment
    containmentPins.digitalWrite(EXPERIMENT, EXPERIMENT_ON);

    // update our state
    state.experimenting = true;
    sm.set_state(state);

    // log that we started the experiment
    lm.log(Bonk::LogType::NOTIFY, "experiment started");
  }
}

void stop_exp() {
  if (state.experimenting) {
    // turn off the experiment
    containmentPins.digitalWrite(EXPERIMENT, EXPERIMENT_OFF);

    // update our state
    state.experimenting = false;
    sm.set_state(state);

    // log that we started the experiment
    lm.log(Bonk::LogType::NOTIFY, "experiment stopped");
  }
}

void collect_data() {
  static unsigned long last_log_time = millis();
  if (millis() - last_log_time >= LOG_TIME_CUTOFF) {
    float current1 = chip1.readShuntCurrent();
    float voltage1 = chip1.readShuntVoltage();
    float current2 = chip2.readShuntCurrent();
    float voltage2 = chip2.readShuntVoltage();
    uint16_t local_temperature = thermometer.readLocalTemperature();
    uint16_t remote_temperature = thermometer.readRemoteTemperature();
//    data_file.printField(voltage1, ',', 4);
//    data_file.printField(current1, ',', 4);
//    data_file.printField(voltage2, ',', 4);
//    data_file.printField(current2, ',', 4);
//    data_file.printField(local_temperature, ',');
//    data_file.printField(remote_temperature, '\n');
    // data_file.flush();    // TODO: might take too much time, needs characterization
    last_log_time = millis();
  }
}

void start_drain() {
  if (!state.solenoid_3_open && !state.pump_powered) {
    // open the drain valve
    containmentPins.digitalWrite(SOL_3, SOL_ON);
    containmentPins.digitalWrite(PUMP_TOGGLE, PUMP_2);
    containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_ON);

    // update our state
    state.solenoid_3_open = true;
    state.pump_toggle = true;
    state.pump_powered = true;
    sm.set_state(state);

    // log that we opened the solenoid
    lm.log(Bonk::LogType::NOTIFY, "drain started");
  }
}

void stop_drain() {
  if (state.pump_powered && state.solenoid_3_open) {
    // turn off the drain pump
    containmentPins.digitalWrite(PUMP_POWER, PUMP_POWER_ON);
    containmentPins.digitalWrite(SOL_3, SOL_OFF);

    // update our state
    state.pump_powered = false;
    state.solenoid_3_open = false;
    sm.set_state(state);

    // log that we turned off the drain pump
    lm.log(Bonk::LogType::NOTIFY, "drain stopped");
  }
}

void tick_fsm(Bonk::FlightEvent fe) {
  // FSM Transitions
  Serial.println((int)fe);
  switch (state.current_action) {
    case action::IDLE:
      if (fe == Bonk::FlightEvent::Liftoff) {
        state.current_action = action::START_SULF;
      } else if (fe == Bonk::FlightEvent::CoastStart) {
        state.current_action = action::START_EXP;
      } else {
        state.current_action = action::IDLE;
      }
      break;

    case action::START_SULF:
      if (state.pump_powered && state.solenoid_1_open) {
        state.current_action = action::CHECK_FULL;
      } else {
        state.current_action = action::START_SULF;
      }
      break;

    case action::STOP_SULF:
      if (!state.pump_powered && !state.solenoid_1_open) {
        state.current_action = action::IDLE;
      } else {
        state.current_action = action::STOP_SULF;
      }
      break;

    case action::CHECK_FULL:
      if (check_full()) {
        if (state.last_action == action::START_SULF) {
          state.current_action = action::STOP_SULF;
        } else if (state.last_action == action::START_WATER) {
          state.current_action = action::STOP_WATER;
        }
      } else {
        state.current_action = action::CHECK_FULL;
      }
      break;
      
    case action::START_EXP:
      if (state.experimenting) {
        state.current_action = action::COLLECT_DATA;
      } else {
        state.current_action = action::START_EXP;
      }
      break;

    case action::COLLECT_DATA:
      if (fe == Bonk::FlightEvent::CoastEnd) {
        state.current_action = action::STOP_EXP;
      } else {
        state.current_action = action::COLLECT_DATA;
      }
      break;

    case action::STOP_EXP:
      if (!state.experimenting) {
        state.current_action = action::START_DRAIN;
      } else {
        state.current_action = action::STOP_EXP;
      }
      break;

    case action::START_DRAIN:
      if (state.pump_toggle && state.pump_powered) {
        state.current_action = action::DRAIN_WAIT;
      } else {
        state.current_action = action::START_DRAIN;
      }
      break;

    case action::DRAIN_WAIT:
      static unsigned long drain_time;
      if (state.last_action == action::START_DRAIN) {
        drain_time = millis();
      }

      if (millis() - drain_time >= DRAIN_TIME_CUTOFF) {
        state.current_action = action::CHECK_DRAIN;
      } else {
        state.current_action = action::DRAIN_WAIT;
      }
      break;

    case action::CHECK_DRAIN:
      if (check_drain()) {
        state.current_action = action::STOP_DRAIN;
      } else {
        state.current_action = action::CHECK_DRAIN;
      }
      break;

    case action::STOP_DRAIN:
      if (state.pump_toggle && !state.pump_powered) {
        if (state.rinses == RINSES) {
          state.current_action = action::IDLE;
        } else {
          state.current_action = action::START_WATER;
        }
      } else {
        state.current_action = action::STOP_DRAIN;
      }
      break;

    case action::START_WATER:
      if (!state.pump_toggle && state.pump_powered) {
        state.current_action = action::CHECK_FULL;
      } else {
        state.current_action = action::START_WATER;
      }
      break;

    case action::STOP_WATER:
      if (!state.pump_powered) {
        state.current_action = action::START_DRAIN;
      } else {
        state.current_action = action::STOP_WATER;
      }
      break;
  }

  // FSM Actions
  switch (state.current_action) {
    case action::START_SULF:
      start_sulf();
      break;

    case action::STOP_SULF:
      stop_sulf();
      break;

    case action::START_EXP:
      start_exp();
      break;

    case action::COLLECT_DATA:
      collect_data();
      break;

    case action::STOP_EXP:
      stop_exp();
      break;

    case action::START_DRAIN:
      start_drain();
      break;

    case action::STOP_DRAIN:
      stop_drain();
      break;

    case action::START_WATER:
      start_water();
      break;

    case action::STOP_WATER:
      stop_water();
      break;
  }
  
  if (state.last_action != state.current_action) {
    sm.set_state(state);
    Serial.println(state.current_action);
  }
  state.last_action = state.current_action;
}

void loop() {
  // tick BONK
  eh.tick();

  // get the most recent New Shepard flight event
  Bonk::ShipReading last_reading = eh.getLastReading();

  // tick the controller FSM
  tick_fsm(last_reading.event);
}
